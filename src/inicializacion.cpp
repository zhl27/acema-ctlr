#include "main.h"
#include "config.h"
#include <freertos/projdefs.h>


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

int contadorMde = 0;
int contadorFlash = 0;
int contadorSensores = 0;
int contadorLora = 0;


static CmdDispatcher cmdDispatcher;
// int muestreo_datos_crudos_ms = 500; // cada 0,5 segundos


void initSerialLog(){
    Serial.begin(115200); // TODO: Para la Compu de vuelo no se usa Serial

    while (!Serial)
        delay(1000);

    ESP_LOGI("SETUP", "Comenzando SETUP.");
}

bool initHardware(){
    bool inicializacion = false;
    // Inicializa internamente mpu y bpm, + la calibración
    inicializacion = Sensors::init();
    if(inicializacion){
        ESP_LOGI("init hard", "Inicialización de Sensors exitosa");
    }else{
        ESP_LOGE("init hard", "ERROR en la inicializacion de sensors");
    }

    inicializacion = Actuators::init();
    
    if(inicializacion){
        ESP_LOGI("init hard", "Inicialización de Actuators exitosa");
    }else{
        ESP_LOGE("init hard", "ERROR en la inicializacion de Actuators");
    }
    GSE::init();
    cmdDispatcher.init();

    return inicializacion;
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

// acá se realiza la depuración de los datos.
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
    kalmanPitch.deshabilitarVarianzaDinamica();
    kalmanYaw.deshabilitarVarianzaDinamica();

    while (true)
    {
        if (xQueueReceive(xColaSensores, &raw, portMAX_DELAY) == pdTRUE){

            //print_data_raw(&raw);

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
            #ifdef DEBUG_DATOS_CRUDOS
            const float accelX_m_s2 = raw.mpu.accel_x_m_s2;
            const float accelY_m_s2 = raw.mpu.accel_y_m_s2;
            const float accelZ_m_s2 = raw.mpu.accel_z_m_s2;
            #else 
            const float accelX_m_s2 = raw.mpu.accel_x_m_s2;
            const float accelY_m_s2 = raw.mpu.accel_z_m_s2;
            const float accelZ_m_s2 = raw.mpu.accel_y_m_s2;
            #endif
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
            const float cos_inc = cosf(pitch_rad) * cosf(yaw_rad); 
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
            //const float accelVertical_m_s2 = accelVertical_g * G_TO_MS2;

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
                    
                    Serial.printf("[Filtro] Calibración de rampa lista. Altitud ASL: %.2f m\n", altitud_rampa_m);
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
            //constexpr float RAD_TO_DEG = 57.2957795131f;

            data_all_t out = {};

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
            out.vel_z_filtrada_m_s  = kalmanAlt.getVelocity(); // TODO: Tomar a Y como eje vertical. Por ahora, para testeos Z es eje vertical. DEBEMOS CAMBIARLO.
            out.aceleracion_z_m_s2  = accelVertical_m_s2;

            // Ambientales
            out.temperatura_amb_c   = raw.bmp.temp_deg_c;
            out.densidad_aire_kg_m3 = emaDensidad.actualizar(calcularDensidadAire(raw.bmp.presion_hpa, raw.bmp.temp_deg_c));

            // GPS
            out.gps_is_valid = raw.gps.is_valid;
            out.gps_hdop = raw.gps.hdop;
            out.gps_nro_satelites = raw.gps.satellites;
            out.gps_latitud = raw.gps.latitude;
            out.gps_longitud = raw.gps.longitude;

            // out.altitud_rampa_asl_m = altitud_rampa_m;

//            print_data(&out);
//            plot_all_processed(&out);
            plot_actitud_filtrada(&out);
            plot_cinematica_filtrada(&out);
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

            EnlaceGSE::enviarTelemetria(out);
        }
        else {
            ESP_LOGI(TAG_TASK_DATA_FILTER, "No messages (timeout)");
        }
    }
}

// Mock implementation of the State Machine task: consumes sensor messages and forwards/acts on them
void vTaskStateMachine(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_STATE_MACHINE, "Core ID: %d", xPortGetCoreID());

        size_t item_size = 0;
        // 1. Receive as a generic void pointer
        void *item = xRingbufferReceive(xStateMachineRingbuf, &item_size, pdMS_TO_TICKS(2000));

        if (item != NULL) {
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
            ESP_LOGI(TAG_TASK_STATE_MACHINE, "No messages (timeout)");
        }

        // Le permite al scheduler del RTOS resetear el WATCHDOG
        vTaskDelay(pdMS_TO_TICKS(5));
    }

}

// Mock implementation of the Flash task: consumes items from the Flash ringbuffer and "persists" them
void vTaskFlash(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_FLASH, "Core ID: %d", xPortGetCoreID());

        size_t item_size = 0;
        void *item = xRingbufferReceive(xFlashRingbuf, &item_size, pdMS_TO_TICKS(5000));

        if (item != NULL) {
            if (item_size == sizeof(data_all_t)) {

                data_all_t *datos_sensores = static_cast<data_all_t *>(item);

                // Print a specific member of the struct (like elapsed_time) instead of %s
                // Serial.printf("[Flash] Persisting (%d bytes). Time: %lu\n", static_cast<int>(item_size), micros());
                // SerialPrint::plot("contadorFlash", contadorFlash);
                // contadorFlash++;
                // TODO: In a real implementation, write 'datos' to SD/flash.
            }
            vRingbufferReturnItem(xFlashRingbuf, item);
        } else {
            ESP_LOGI(TAG_TASK_FLASH, "No items to persist (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void vTaskLora(void *pvParameters) {
    (void)pvParameters;
    pkt_t rxPacket; // Para almacenar paquetes entrantes

    while (true) {
        size_t item_size = 0;
        void *item = xRingbufferReceive(xLoraRingbuf, &item_size, pdMS_TO_TICKS(10)); // Timeout bajo para no bloquear Rx

        if (item != NULL) {
            // Validamos que sea el tamaño de nuestro envoltorio
            if (item_size == sizeof(TxEnvelope_t)) {
                
                TxEnvelope_t *sobre = static_cast<TxEnvelope_t *>(item);

                // Solo pasamos a GSE::actualizar si el enlace está CONNECTED
                if (GSE::estado_conexion_gse() == ROCKET_CONNECTED) {
                    
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

                        // case Protocolo::C_ACK:
                        //     GSE::enviar_ack(&(sobre->payload.ack));
                        //     break;
                    }
                }
            }
            vRingbufferReturnItem(xLoraRingbuf, item);
        }

        // Aquí también iría la lógica (explicada en el mensaje anterior) 
        // para lora.read_paquete() y recibir comandos (G_CMD) sin bloqueos.
        // ---------------------------------------------------------
        // 1. FASE RX: Escuchar comandos desde la estación terrena
        // ---------------------------------------------------------
        if (GSE::leer_paquete(&rxPacket)) {
            if (rxPacket.protocol == lora_protocol::G_CMD) {
                // Casteamos el payload a nuestra estructura de comando
                // Asumiendo que el GSE envió un CommandPayload { uint32_t opCode; float value; }
                CommandPayload* cmd = static_cast<CommandPayload*>(rxPacket.payload);
                
                ESP_LOGI(TAG_TASK_LORA, "[Lora] Comando Recibido: OP=%d, VAL=%.2f\n", cmd->opCode, cmd->value);
                
                // Encolamos el comando en el CmdDispatcher
                cmdDispatcher.enqueueCommand(cmd->opCode, cmd->value);
            }
        }
        // GSE::mantener_conexion() // Podrías extraer el switch(ROCKET_INIT...) a un método que se llame cíclicamente aquí.
    }
}




// ==========================================
// LÓGICA DE COMANDOS
// ==========================================

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

bool registrarComandos(){

    bool state;
    state = cmdDispatcher.registerCommand(
        CMD_DISPARAR_PIRO, 
        comando_disparar_piro, 
        &Actuators::getPyroDrogue() 
    );
    return state;
}