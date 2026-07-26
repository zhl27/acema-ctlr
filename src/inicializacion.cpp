#include "main.h"
#include "config.h"
#include <freertos/projdefs.h>


static const char *TAG_MAIN = "MAIN_SETUP";
bool DEBUG_SERIAL = true;
using namespace ConfigInit;

// Instancias globales de los filtros (Ajustar las varianzas empíricamente. Ej: Gyro=0.001, Accel=0.01)
static Kalman1D kalmanPitch(VARIANZA_INICIAL_GIROSCOPIO, VARIANZA_INICIAL_ACELEROMETRO);
static Kalman1D kalmanYaw(VARIANZA_INICIAL_GIROSCOPIO, VARIANZA_INICIAL_ACELEROMETRO);
static Kalman2D kalmanAlt;


static EmaFilter emaTemperatura(FREC_SAMPLING_SENSORS_HZ, EMA_FREC_CORTE_TEMPERATURA);
static EmaFilter emaPresion(FREC_SAMPLING_SENSORS_HZ, EMA_FREC_CORTE_PRESION_ATM);
static EmaFilter emaDensidad(FREC_SAMPLING_SENSORS_HZ, EMA_FREC_CORTE_DENSIDAD_AIRE);
static EmaFilter emaAccelVertical(FREC_SAMPLING_SENSORS_HZ, EMA_FREC_CORTE_ACCEL_VERTICAL);

// Ring buffer handles
RingbufHandle_t xStateMachineRingbuf;
RingbufHandle_t xLoraRingbuf;
RingbufHandle_t xFlashRingbuf;

// Cola simple 
QueueHandle_t xColaSensores;

// Task Function Prototypes
void vTaskReadSensors(void *pvParameters);      // la tarea que lee los sensores y envía los datos a la cola de datos crudos
void vTaskDataFilter(void *pvParameters);       // agarra los datos crudos de los sensores, los procesa y los distribuye
void vTaskStateMachine(void *pvParameters);     // la máquina de estados que orquesta la lógica principal del cohete, incluyendo la gestión de estados de conexión, envío de telemetría, etc.
void vTaskFlash(void *pvParameters);            // la caja negra que persiste cada dato entrante.
void vTaskLora(void *pvParameters);             // maneja la comunicación LoRa, incluyendo el envío de datos y la gestión de la conexión con el GSE.

static const char *TAG_TASK_SENSORS = "TASK SENSORS";
static const char *TAG_TASK_DATA_FILTER = "TASK DATA FILTER";
static const char *TAG_TASK_STATE_MACHINE = "TASK STATE MACHINE";
static const char *TAG_TASK_FLASH = "TASK FLASH";
static const char *TAG_TASK_LORA = "TASK LORA";



mFlash cajaNegra(FLASH_CS);
// !< Acá se guardan los datos de configuración inicial
ConfigDatos g_configActual = {}; // <--- DEBE SER GLOBA U EN UNA STRUCT GLOBAL (SYSTEM)


int contadorMde = 0;
int contadorFlash = 0;
int contadorSensores = 0;
int contadorLora = 0;


static CmdDispatcher cmdDispatcher;
mBuzzer buzzer (BUZZER_PIN);

// int muestreo_datos_crudos_ms = 500; // cada 0,5 segundos

//>! Orden

// 2. Para cargar datos de configuración
bool init_black_box()
{
    if(!cajaNegra.begin(sizeof(data_all_t))){
        if(EnlaceGSE::enviarError("El Flash no se ha inicializado correctamente"))
        {
            ESP_LOGI("INIT_BLACK_BOX", "El Flash no se ha inicializado correctamente");
        }
        else {
            ESP_LOGE("INIT_BLACK_BOX", "EnlaceGSE roto");
        }
        return false; // false? sep, porque malió sal
    }

    if(!cajaNegra.cargarConfig(&g_configActual, sizeof(g_configActual))){
        if(EnlaceGSE::enviarError("No se pudo cargar la configuracion guardada en Flash"))
        {
            ESP_LOGI("INIT_BLACK_BOX","No se pudo cargar la configuracion guardada en Flash");
        }
        else {
            ESP_LOGE("INIT_BLACK_BOX", "EnlaceGSE roto");
        }
        return false; // false? sep, porque malió sal
    }
    return true;
}

bool init_hardware() {
    bool inicializacion = false;

    // NOTA: Sensors::init() AHORA SOLO DEBE INICIALIZAR LOS BUSES Y OBJETOS (begin).
    // LA CALIBRACIÓN DEBE HABER SIDO EXTRAÍDA DE ESTE MÉTODO.
    inicializacion = Sensors::init();
    if(inicializacion) {
        ESP_LOGI("INIT_HARD", "Inicialización de Sensors exitosa (Sin calibrar)");
    } else {
        ESP_LOGE("INIT_HARD", "ERROR en la inicializacion de sensors");
        return false;
    }

    inicializacion = Actuators::init();
    if(inicializacion) {
        ESP_LOGI("INIT_HARD", "Inicialización de Actuators exitosa");
    } else {
        ESP_LOGE("INIT_HARD", "ERROR en la inicializacion de Actuators");
        return false;
    }

    cmdDispatcher.init();
    return true;
}


// Se esperaría en algun momento, limpiar el logger antes de despegar,
// mas no, que sea condición para el depegue
// LOGICA DE MAQUINA DE ESTADOS: Esto puede ir en la FSM para prevenir los reinicios
void run_boot_logic(const bool sensores_inicializaron_bien) {
    ESP_LOGI("BOOT_LOGIC", "sensores_inicializaron_bien=%d", sensores_inicializaron_bien);

    if(!sensores_inicializaron_bien){
        g_configActual.estado_cohete_actual = Cohete::ST_INIT; // comenzamos MdE desde cero.
        return;
    }
    ESP_LOGI("BOOT_LOGIC", "Evaluando estado de vuelo post-reinicio...");

    constexpr mMPU6050::GravityAxis nuestro_eje_vertical = mMPU6050::GravityAxis::PLUS_Y;

    // INTERLOCK 1: ¿La misión había sido armada/iniciada antes del reinicio?
    if (g_configActual.mision_activa == true) {

        // INTERLOCK 2: Validación por sensores (se asume que Sensors::get_raw_data obtiene una lectura válida)
        data_raw_t raw_data = {};
        float altitud_actual = {};
        // Descarta ruido inicial
        for (size_t i = 0; i < 50; i++)
        {
            raw_data = Sensors::get_raw_data();
            altitud_actual = raw_data.bmp.altitud_snm_m;
        }

        // Comparamos la altitud actual con la altitud base guardada en flash
        if ((altitud_actual - g_configActual.altitud_base_agl) > 20.0f) {
            // ¡ESTAMOS EN EL AIRE REALMENTE! Recuperando vuelo.
            ESP_LOGW("BOOT_LOGIC", "¡Reinicio en vuelo detectado! Saltando calibración IMU.");
            g_configActual.estado_cohete_actual = Cohete::ST_BOOST;

            // Aquí NO se llama a Sensors::calibrar(). El filtro dependerá de los offsets guardados
            // previamente en el flash, o usará valores por defecto seguros.
            math::Vector3f biasAccel = g_configActual.bias.accel;
            math::Vector3f biasGyro = g_configActual.bias.gyro;
            Sensors::getMPU6050().setBias(biasAccel,biasGyro);
            // NOTE: no importa setear el eje de gravedad pues eso se quedó guardado en la bias

        } else {
            // Falsa alarma. Se armó, pero nunca despegó (o ya aterrizó).
            ESP_LOGI("BOOT_LOGIC", "Misión activa pero en tierra. Calibrando sensores...");
            g_configActual.estado_cohete_actual = Cohete::ST_INIT;
            mMPU6050::CalibrationStatus result = Sensors::getMPU6050().calibrar(nuestro_eje_vertical); // <-- Calibración segura en tierra
            g_configActual.bias.accel = Sensors::getMPU6050().get_calibration_result().accel_bias;
            g_configActual.bias.gyro = Sensors::getMPU6050().get_calibration_result().gyro_bias;
        }

    } else {
        // No hay misión activa. Arranque normal.
        ESP_LOGI("BOOT_LOGIC", "Arranque normal de prevuelo. Calibrando sensores...");
        g_configActual.estado_cohete_actual = Cohete::ST_INIT;
        mMPU6050::CalibrationStatus result =  Sensors::getMPU6050().calibrar(nuestro_eje_vertical); // <-- Calibración segura en tierra
        g_configActual.bias.accel = Sensors::getMPU6050().get_calibration_result().accel_bias;
        g_configActual.bias.gyro = Sensors::getMPU6050().get_calibration_result().gyro_bias;
    }
}


void init_lora(){
    // Crea el buffer de telemetria
    xLoraRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xLoraRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xLoraRingbuf");

    // xFlashRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    // if (xFlashRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xFlashRingbuf");

    // 5Inicializar la capa de enlace GSE
    ESP_LOGI(TAG_MAIN, "Enlace LoRa/GSE configurado.");

    GSE::init();
    EnlaceGSE::inicializar(xLoraRingbuf);
}





constexpr float R_AIR = 287.05f;

inline float calcularDensidadAire(const float pressure_hpa, const float temperature_deg_c)
{
    return (pressure_hpa ) / (R_AIR * (temperature_deg_c + 273.15f));
}


// ==========================================
// LÓGICA DE LAS TAREAS
// ==========================================

// TODO: Pensar sobre este texto: "You need to gather large bursts of hardware data inside an Interrupt Service Routine (ISR) to be processed later by a task."
void vTaskReadSensors(void *pvParameters) {
    // const TickType_t xFrequency = pdMS_TO_TICKS(7); // TODO: ~6.7 ms → 150 Hz --> frecuencia de rafagas --> es en realidad req de vTaskLora
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    // ---------------------------------------------------------------------------------
    // Timer de muestreo
    // ---------------------------------------------------------------------------------
    TickType_t xLastWakeTime;
    const TickType_t xPeriodo = pdMS_TO_TICKS(PERIOD_SAMPLING_SENSORS_MS); // Muestreo cada 10 ms (100 Hz)

    // Inicializar el tiempo de referencia para vTaskDelayUntil
    xLastWakeTime = xTaskGetTickCount();

    (void)pvParameters;
    while (true) {
        // Espera estricta y precisa hasta el próximo ciclo de 10ms --> ademas nos permite procesar a las otras Tasks
        vTaskDelayUntil(&xLastWakeTime, xPeriodo);

        ESP_LOGD(TAG_TASK_SENSORS, "Core ID: %d", xPortGetCoreID());

        // TODO: IMPLEMENTAR ESTA IDEA PARA LOS mockBMP280 y mockMPU6050
        // data_raw_t raw;
        // if(DEBUG_SERIAL){
        //     // PARA SIMULAR DATOS POR SERIAL
        // }
        // else{
        //     raw = Sensors::get_raw_data(); // si no simulamos, usamos datos reales.
        // }
        // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
        data_raw_t raw = Sensors::get_raw_data();
        //print_data_raw(&raw);

        // TODO: Curiosidad: Por qué se utiliza una Queue en lugar de un Ringbuffer ?

        // 3. Enviar a la cola del Filtro de Kalman de forma NO bloqueante (Timeout = 0)
        // Si la cola se llena porque la se retrasó, preferimos perder una muestra
        // antes que congelar el temporizador de 10ms de los sensores.
        if (xColaSensores != NULL) {
            if (xQueueSend(xColaSensores, &raw, 0) != pdTRUE) {
                // Si falla, registramos un Warning de telemetría (la cola está saturada)
                ESP_LOGW(TAG_TASK_SENSORS, "Cola llena: Muestra descartada para proteger el timing.");
                // SI VEMOS ESTE ERROR, HAY QUE AUMENTAR EL TAMÑO DE LA COLA
            }
        }
        // if (xDataFilterRingbuf != NULL) {
        //     if (xRingbufferSend(xDataFilterRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(50)) != pdTRUE) {
        //         ESP_LOGE(TAG_TASK_SENSORS, "xRingbufferSend -> xDataFilterRingbuf failed (raw)");
        //     }
        // }

        // SerialPrint::plot("contadorSensores", contadorSensores);
        // contadorSensores++;

        // xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


//============================================================

// Función inline para acotar valores (reemplazo de std::clamp)
inline float mi_clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// ----------------------------------------------------------------------
// Tarea de filtrado modificada
// ----------------------------------------------------------------------
void vTaskDataFilter(void *pvParameters)
{
    (void) pvParameters;
    // ----------------------------------------------------------------------
    // 1. VARIABLES DE ESTADO PARA CALIBRACIÓN 
    // ----------------------------------------------------------------------
    static bool kalmanAltInit = false;
    static uint16_t baro_calib_muestras = 0;
    static float suma_altitud_snm = 0.0f;
    static float altitud_rampa_m = 0.0f;
    constexpr uint16_t MUESTRAS_CALIBRACION_BARO = 100; // Ej: 100 muestras a 10ms = 1 segundo de calibración
    constexpr float G_TO_MS2 = 9.80665f;

    data_raw_t raw;
    int64_t timestampAnterior = 0;
    bool primeraMuestra = true;

    // HABILITA LA CORRECIÓN DEL SENSOR IMU POR LA ACCELERACIÓN EN LA ETAPA DE BOOST
    // DESABILITAR SI SE QUIERE TESTEAR EN BANCO
    // kalmanPitch.deshabilitarVarianzaDinamica();
    // kalmanYaw.deshabilitarVarianzaDinamica();

    kalmanPitch.habilitarVarianzaDinamica();
    kalmanYaw.habilitarVarianzaDinamica();
    while (true)
    {
        if (xQueueReceive(xColaSensores, &raw, portMAX_DELAY) == pdTRUE){

            print_plotter_data_raw(&raw);

            //----------------------------------------------------------------------
            // Primera muestra: solamente inicializa el tiempo
            //----------------------------------------------------------------------
            if (primeraMuestra)
            {
                timestampAnterior = raw.timestamp_us;
                primeraMuestra = false;
                continue;
            }

            //----------------------------------------------------------------------
            // detal t
            //----------------------------------------------------------------------
            const float dt = (raw.timestamp_us - timestampAnterior) * 1e-6f;

            timestampAnterior = raw.timestamp_us;

            //----------------------------------------------------------------------
            // MAPEO DE EJES (Sensor MPU -> Cohete Físico)
            // Todo permanece en las unidades nativas
            //----------------------------------------------------------------------

            const float accelX_m_s2 = raw.mpu.accel_x_m_s2;
            const float accelY_m_s2 = raw.mpu.accel_z_m_s2;
            const float accelZ_m_s2 = raw.mpu.accel_y_m_s2;

            const float gyroRoll_rad_s  = raw.mpu.gyro_z_rad_s;     // Alabeo
            const float gyroPitch_rad_s = raw.mpu.gyro_y_rad_s;     // Cabeceo
            const float gyroYaw_rad_s   = raw.mpu.gyro_x_rad_s;     // Rotación sobre el eje vertical

            //----------------------------------------------------------------------
            // Ángulos obtenidos del acelerómetro (Trigonometría 3D Esférica)
            // Eje vertical en reposo: +X
            //----------------------------------------------------------------------

            // Calcula las magnitudes adyacentes usando Pitágoras
            const float adj_pitch = sqrtf((accelX_m_s2 * accelX_m_s2) + (accelY_m_s2 * accelY_m_s2));
            const float adj_yaw   = sqrtf((accelX_m_s2 * accelX_m_s2) + (accelZ_m_s2 * accelZ_m_s2));

            // Cuando X e Y sean 0, atan2f devolverá 0 radianes (perfectamente vertical).
            const float accelPitch_rad = atan2f(accelX_m_s2, accelZ_m_s2);
            const float accelYaw_rad   = atan2f(accelY_m_s2, accelZ_m_s2);

            //----------------------------------------------------------------------
            // Kalman 1D (Trabaja en rad, rad/s, m/s^2)
            //----------------------------------------------------------------------
            const float pitch_rad = kalmanPitch.update(gyroPitch_rad_s, accelPitch_rad, dt, accelZ_m_s2);
            const float yaw_rad   = kalmanYaw.update(gyroYaw_rad_s, accelYaw_rad, dt, accelZ_m_s2);

            //----------------------------------------------------------------------
            // Inclinación total respecto de la vertical
            //----------------------------------------------------------------------
            float cos_inc = cosf(pitch_rad) * cosf(yaw_rad); 
            
            // Forzamos a que el valor nunca salga del rango [-1.0, 1.0] de forma segura
            cos_inc = mi_clamp(cos_inc, -1.0f, 1.0f);
            
            const float inclinacion_rad = acosf(cos_inc);

            //----------------------------------------------------------------------
            // Proyección de la aceleración longitudinal sobre el eje vertical global
            //----------------------------------------------------------------------
            // Corrección matemática de doble proyección y ajuste de unidades (G a m/s^2)
            float accelVertical_m_s2 = (accelZ_m_s2 * cos_inc) - (9.80665f * cos_inc * cos_inc);

            // Suavizado por filtro de media móvil exponencial
            accelVertical_m_s2 = emaAccelVertical.actualizar(accelVertical_m_s2);


            //----------------------------------------------------------------------
            // Kalman 2D
            //
            // Este filtro trabaja naturalmente en SI.
            //----------------------------------------------------------------------
            if (!kalmanAltInit) {
                // Fase de acumulación: Promedia la altitud antes del vuelo
                if (baro_calib_muestras < MUESTRAS_CALIBRACION_BARO) {
                    suma_altitud_snm += raw.bmp.altitud_snm_m;
                    baro_calib_muestras++;
                    
                    // Evita que el resto de la tarea envíe basura mientras calibra
                    continue;
                } 
                // Fase de inicialización del filtro
                else {
                    altitud_rampa_m = suma_altitud_snm / (float)MUESTRAS_CALIBRACION_BARO;
                    
                    // El filtro arranca estrictamente en 0 metros (AGL) y 0 m/s
                    kalmanAlt.init(0.0f, 0.0f, VARIANZA_INICIAL_ACELEROMETRO, VARIANZA_INICIAL_GIROSCOPIO);  
                    kalmanAltInit = true;
                    
                    Serial.printf("[Filtro] Calibración de rampa lista. Altitud ASL: %f m\n", altitud_rampa_m);
                }
            }

            // Calculamos la altitud relativa al nivel del suelo (AGL)
            const float altitud_agl_m = raw.bmp.altitud_snm_m - altitud_rampa_m;

            // Le inyectamos la altitud AGL al filtro
            kalmanAlt.update(dt, accelVertical_m_s2, altitud_agl_m);

            //----------------------------------------------------------------------
            // EMPAQUETADO
            //
            // Recién acá convertimos a las unidades públicas de data_all_t.
            //----------------------------------------------------------------------
            data_all_t out = {};
            out.timestamp_micros = raw.timestamp_us;
            
            // Serial.print(">heap_libre:");
            // Serial.println(ESP.getFreeHeap());

            // Velocidades angulares
            out.vel_angular_x_deg_s = gyroPitch_rad_s * RAD_TO_DEG;
            out.vel_angular_y_deg_s = gyroRoll_rad_s  * RAD_TO_DEG;
            out.vel_angular_z_deg_s = gyroYaw_rad_s   * RAD_TO_DEG;

            // Actitud
            out.angulo_pitch_deg      = pitch_rad * RAD_TO_DEG;
            out.angulo_yaw_deg        = yaw_rad * RAD_TO_DEG;
            out.angulo_respecto_z_deg = inclinacion_rad * RAD_TO_DEG;

            // Cinemática vertical
            out.altitud_filtrada_m  = kalmanAlt.getAltitude();
            out.velocidad_vertical_filtrada_m_s  = kalmanAlt.getVelocity();
            out.aceleracion_vertical_m_s2  = accelVertical_m_s2;

            // Ambientales
            out.temperatura_amb_c   = raw.bmp.temp_deg_c;
            out.densidad_aire_kg_m3 = emaDensidad.actualizar(calcularDensidadAire(raw.bmp.presion_hpa, raw.bmp.temp_deg_c));

            // GPS
            out.gps_is_valid = raw.gps.is_valid;
            out.gps_hdop = raw.gps.hdop;
            out.gps_nro_satelites = raw.gps.satellites;
            out.gps_latitud = raw.gps.latitude;
            out.gps_longitud = raw.gps.longitude;

            plot_actitud_filtrada(&out);
            plot_cinematica_filtrada(&out);
            
            // Timestamp corregido para evitar fugas de memoria o punteros fantasma
            // Serial.print(">timestamp_ms:");
            // Serial.println((uint32_t)(t / 1000));

            //----------------------------------------------------------------------
            // Distribución (MdE, Lora)
            //----------------------------------------------------------------------
            if (xStateMachineRingbuf != NULL)
            {
                xRingbufferSend(
                    xStateMachineRingbuf,
                    &out,
                    sizeof(data_all_t),
                    0);
            }

            if (xFlashRingbuf != NULL)
            {
                xRingbufferSend(
                    xFlashRingbuf,
                    &out,
                    sizeof(data_all_t),
                    0);
            }
            EnlaceGSE::enviarTelemetria(out);

            print_plotter_data_processed(&out);
        }
        else {
            ESP_LOGI(TAG_TASK_DATA_FILTER, "No messages (timeout)");
        }

        // Le permite al scheduler del RTOS resetear el WATCHDOG
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
// Mock implementation of the State Machine task: consumes sensor messages and forwards/acts on them
void vTaskStateMachine(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_STATE_MACHINE, "Core ID: %d", xPortGetCoreID());

        size_t item_size = 0;
        // 1. Receive as a generic void pointer
        void *item = xRingbufferReceive(xStateMachineRingbuf, &item_size, pdMS_TO_TICKS(5000)); // TODO: podemos implementar un patrón Mailbox.

        if (item != nullptr) {
            if (item_size == sizeof(data_all_t)) {

                data_all_t *datos_sensores = static_cast<data_all_t *>(item);


                // NOTA: Asegurarse de que mde_cohete_actualizar acepte un puntero a data_all_t
                Cohete::mde_cohete_actualizar(datos_sensores);

                // SerialPrint::plot("contadorMde", contadorMde);
                // contadorMde++;
            } else {
                ESP_LOGE(TAG_TASK_STATE_MACHINE, "Tamaño de item no coincide con data_all_t");
            }

            // 4. Free the memory
            vRingbufferReturnItem(xStateMachineRingbuf, item);

        } else {
            ESP_LOGI(TAG_TASK_STATE_MACHINE, "No messages (timeout)"); // TODO: No sería más conveniente quitar el timeout ?
        }

        // Le permite al scheduler del RTOS resetear el WATCHDOG
        vTaskDelay(pdMS_TO_TICKS(5));
    }

}

void vTaskFlash(void *pvParameters) {
    mFlash* ptrCajaNegra = static_cast<mFlash*>(pvParameters);

    ESP_LOGI(TAG_TASK_FLASH, "vTaskFlash iniciada en Core %d", xPortGetCoreID());

    while (true) {
        // -----------------------------------------------------------------
        // 1. CHEQUEO DE NOTIFICACIONES ASÍNCRONAS (No bloqueante)
        // -----------------------------------------------------------------
        const uint32_t notif_val = ulTaskNotifyTake(pdTRUE, 0);

        // A) ACCIÓN CRÍTICA: VOLCADO DE EMERGENCIA DE LA RAM A FLASH
        if (notif_val > 0 || Cohete::SYSTEM.flags.volcar_ram_a_flash) {
            ESP_LOGW(TAG_TASK_FLASH, "Iniciando volcado de seguridad a Flash Externa (mFlash)...");

            size_t item_size = 0;
            void *item = nullptr;

            // DRENADO TOTAL: Vaciamos todo el ring buffer iterativamente
            // con timeout 0 para rescatar cada byte disponible en RAM antes del choque.
            while ((item = xRingbufferReceive(xFlashRingbuf, &item_size, 0)) != NULL) {
                if (item_size == sizeof(data_all_t)) {
                    if (ptrCajaNegra != nullptr) {
                        ptrCajaNegra->guardarPuntoLog(item, sizeof(data_all_t));
                    }
                    else {
                        ESP_LOGE(TAG_TASK_FLASH, "No existe el objeto mFlash");
                        // EnlaceGSE::enviar_error("No existe el objeto mFlash");
                    }
                }
                vRingbufferReturnItem(xFlashRingbuf, item);
            }

            Cohete::SYSTEM.flags.volcar_ram_a_flash = false;
            ESP_LOGI(TAG_TASK_FLASH, "¡Volcado masivo de RAM a Flash completado!");
        }

        // B) ACCIÓN DE MANTENIMIENTO: BORRAR LOG (Independiente de la emergencia)
        if (Cohete::SYSTEM.flags.borrar_log) { //TODO: Tirar otra notificacion desde la MdE
            ESP_LOGW(TAG_TASK_FLASH, "Ejecutando borrado de log en Flash (Caja Negra)...");

            if (ptrCajaNegra != nullptr) {
                ptrCajaNegra->resetearLog();
                Cohete::SYSTEM.flags.borrar_log = false;
                Cohete::SYSTEM.flags.flash_log_borrado = true;
                ESP_LOGI(TAG_TASK_FLASH, "¡Log borrado con éxito!");
            } else {
                ESP_LOGE(TAG_TASK_FLASH, "No existe el objeto mFlash");
            }
        }

        // -----------------------------------------------------------------
        // 2. PERSISTENCIA NORMAL DE DATOS DE SENSORES
        // -----------------------------------------------------------------
        size_t item_size = 0;
        void *item = xRingbufferReceive(xFlashRingbuf, &item_size, pdMS_TO_TICKS(50));

        if (item != NULL) {
            if (item_size == sizeof(data_all_t)) {
                ptrCajaNegra->guardarPuntoLog(item, sizeof(data_all_t));
            }
            vRingbufferReturnItem(xFlashRingbuf, item);
        }
    }
}


void vTaskLora(void *pvParameters) {
    (void)pvParameters;
    pkt_t rxPacket;

    static uint32_t ultimo_ping_millis = 0;
    const uint32_t INTERVALO_PING_MS = 2000; // Intento de reconexión cada 2s si no hay enlace

    ESP_LOGI(TAG_TASK_LORA, "vTaskLora iniciada correctamente.");

    while (true) {

        // ---------------------------------------------------------
        // 0. MODO EMERGENCIA: CATASTROFE (Baliza SOS)
        // ---------------------------------------------------------
        if (Cohete::SYSTEM.flags.emergencia_fatal) {
            // Empaquetamos la última coordenada GPS válida
            char sos_msg[64];
            snprintf(sos_msg, sizeof(sos_msg), "[SOS] LAT:%f LON:%f",
                     Cohete::SYSTEM.ctx_fisico.gps_ultima_latitud_valida,  //TODO: se podría eliminar "datos_actuales.gps_latitud" y usar los datos que recibe de prepo el vTaskLora
                     Cohete::SYSTEM.ctx_fisico.gps_ultima_longitud_valida);

            GSE::enviar_mensaje(sos_msg); // Reutilizamos tu función de C_MGS
            ESP_LOGW(TAG_TASK_LORA, "[LoRa TX] ¡Transmitiendo Baliza SOS!");

            // Bombardear el espectro cada 250ms (Ignora todo lo demás)
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        // ---------------------------------------------------------
        // 1. FASE TX: Transmisión Continua (Independiente del estado)
        // ---------------------------------------------------------
        // NO condicionamos por ROCKET_CONNECTED. Si hay un sobre en la cola,
        // lo transmitimos por aire. Si la GSE se desconecta y se vuelve a conectar,
        // capturará la señal de inmediato sin requerir re-negociación.
        size_t item_size = 0;
        void *item = xRingbufferReceive(xLoraRingbuf, &item_size, pdMS_TO_TICKS(10)); // Timeout bajo para no bloquear Rx

        if (item != NULL) {
            if (item_size == sizeof(TxEnvelope_t)) {
                TxEnvelope_t *sobre = static_cast<TxEnvelope_t *>(item);

                switch(sobre->tipo) {
                    case lora_protocol::C_PLOT:
                        GSE::actualizar_graficas(&(sobre->payload.telemetria));
                        break;

                    case lora_protocol::C_MGS:
                        GSE::enviar_mensaje(sobre->payload.texto);
                        break;

                    case lora_protocol::C_ERR:
                        GSE::enviar_error(sobre->payload.texto);
                        break;

                    case lora_protocol::C_ACK:
                        GSE::enviar_ack(sobre->payload.ack);
                        break;
                }
            }
            vRingbufferReturnItem(xLoraRingbuf, item);
        }

        // ---------------------------------------------------------
        // 2. FASE RX: Procesar paquetes entrantes (Comandos / Handshake)
        // ---------------------------------------------------------
        if (GSE::leer_paquete(&rxPacket)) {
            switch (rxPacket.protocol) {

                // --- COMANDOS DESDE LA GSE ---
                case lora_protocol::G_CMD: {
                    CommandPayload* cmd = static_cast<CommandPayload*>(rxPacket.payload);
                    ESP_LOGI(TAG_TASK_LORA, "[LoRa RX] Comando recibido -> OP: %d, VAL: %.2f", cmd->opCode, cmd->value);
                    cmdDispatcher.enqueueCommand(cmd->opCode, cmd->value);
                    break;
                }

                // --- PING / PONG DE LA ESTACIÓN TERRENA ---
                case lora_protocol::PING:
                case lora_protocol::PONG: {
                    ESP_LOGI(TAG_TASK_LORA, "[LoRa RX] PING/PONG recibido de GSE. Enlace confirmado.");

                    // Actualizamos el estado interno y le avisamos a la MdE
                    GSE::set_estado_conexion(ROCKET_CONNECTED);
                    // Cohete::SYSTEM.flags.gse_conectado = true; // La MdE es la UNICA que puede modificar los datos SYSTEM. Todos los DEMÁS solo tienen permitida LECTURA. En este caso, podemos utilizar en su lugar: get_estado_conexion()

                    // Si fue un PING explícito de la GSE, confirmamos recepción
                    if (rxPacket.protocol == lora_protocol::PING) {
                        GSE::enviar_pong();
                    }
                    break;
                }

                default:
                    ESP_LOGD(TAG_TASK_LORA, "[LoRa RX] Protocolo no manejado: %d", rxPacket.protocol);
                    break;
            }
        }

        // ---------------------------------------------------------
        // 3. FASE HANDSHAKE: Emisión de PING si estamos buscando conexión
        // ---------------------------------------------------------
        if (GSE::get_estado_conexion() != ROCKET_CONNECTED) {
            if (millis() - ultimo_ping_millis >= INTERVALO_PING_MS) {
                ultimo_ping_millis = millis();
                ESP_LOGI(TAG_TASK_LORA, "[LoRa TX] Emitiendo PING de búsqueda de GSE...");

                // Emite el ping de reconexión por RF
                GSE::enviar_mensaje("ROCKET_PING");
            }
        }

        // Ceder control a FreeRTOS para no bloquear el CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



// ==========================================
// LÓGICA DE COMANDOS
// ==========================================

// Callback unificado para apertura de paracaídas (Drogue o Principal)
CmdResult cmd_apertura_paracaidas(float value, void* context) {
    mPyro* piro = static_cast<mPyro*>(context);

    // 1. Evaluar estado de salud (continuidad) sin que sea condición bloqueante
    bool tiene_continuidad = piro->tieneContinuidad();

    // 2. Enviar reporte (ACK) explícito del estado de salud
    // Si la continuidad es OK devuelve 1, si falla devuelve 0.
    // (Ajusta el método según tu clase, ej: EnlaceGSE::enviar_ack() o EnlaceGSE::enviarACK())
    // EnlaceGSE::enviar_ack(tiene_continuidad ? 1 : 0);

    // 3. Ejecutar la mini rutina de despliegue incondicionalmente
    piro->armar();
    piro->disparar((uint32_t)value); // Usamos el value recibido como tiempo de ignición (ms)

    // 4. Retornar el resultado para que el dispatcher cierre la transacción
    int8_t stat =( tiene_continuidad ? (int8_t)1 : (int8_t)0);
    return CmdResult{.status = stat, .data = value};
}

// Callback para ajustar el ángulo del servo (Clamp 0° - 30°)
CmdResult cmd_set_angulo_servo(float value, void* context) {
    // Limitamos el rango para proteger la estructura mecánicamente
    float angulo_seguro = mi_clamp(value, 0.0f, 30.0f);

    // Actuamos directamente sobre el actuador
    Actuators::getServo().sendAngulo(angulo_seguro);

    // Devolvemos status 1 (Éxito) y el ángulo final aplicado
    return CmdResult{.status = 1, .data = angulo_seguro};
}

CmdResult comandoOnPiro(float value, void* context){
    return {1, 0.0f}; // Status OK temporal
}

CmdResult comandoOffPiro(float value, void* context){
    return {1, 0.0f}; // Status OK temporal
}

CmdResult comando_disparar_piro(float value, void* context){
    mPyro* p = static_cast<mPyro*>(context);
    
    bool ok = p->disparar((uint32_t)value);
    
    CmdResult res;
    res.status = ok ? 1 : 0;
    res.data = 0.0f; // No hay dato que devolver en un disparo
    
    return res;
}

CmdResult comandoDesplegarDrogue(float value, void* context){
    return {1, 0.0f}; // Status OK temporal
}

CmdResult cmd_borrar_log(float value, void* context) {
    ESP_LOGI("CMD", "Solicitud de borrado de memoria recibida por LoRa.");

    // Indica la acción
    Cohete::SYSTEM.flags.flash_log_borrado = false;
    Cohete::SYSTEM.flags.borrar_log = true; // manda accion para borrar log

    // Despierta a vTaskFlash INMEDIATAMENTE
    if (Cohete::SYSTEM.procesos.xTaskFlashHandle != NULL) {
        xTaskNotifyGive(Cohete::SYSTEM.procesos.xTaskFlashHandle);
    }

    return CmdResult{.status = 1, .data= 0.0f}; // Respuesta rápida a la estación terrena
}

bool registrar_comandos_gse(){

    bool state = true;
    state &= cmdDispatcher.registerCommand(CMD_CLEAR_LOG, cmd_borrar_log, nullptr);

    // Nuevos comandos
    // Pasamos las instancias correctas a través del puntero de contexto (void* context)
    state &= cmdDispatcher.registerCommand(CMD_DESPLEGAR_DROGUE, cmd_apertura_paracaidas, &Actuators::getPyroDrogue());
    state &= cmdDispatcher.registerCommand(CMD_DESPLEGAR_MAIN, cmd_apertura_paracaidas, &Actuators::getPyroPpal());

    // El servo no necesita pasar por contexto si se accede vía singleton o getter estático
    state &= cmdDispatcher.registerCommand(CMD_SET_SERVO, cmd_set_angulo_servo, nullptr);

    return state;
}