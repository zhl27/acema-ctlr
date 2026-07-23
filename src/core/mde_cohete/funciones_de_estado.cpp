#include "funciones_de_estado.h"

//#include "mde_cohete.h"
#include "SerialPrint.h"
#include "esp_log.h"
/*
#include "services/Actuators.h"
#include "services/GSE.h"
#include "services/Sensors.h"
*/
#include "main.h"
//#include "include.h"
#include "inicializacion.h"

// TODO: Integrar con la variable global COHETE para transiciones de estado

constexpr float     A_GRAV                                  = 9.81;
constexpr float     UMBRAL_ACEL_BOOST                       = 2 * A_GRAV;   // 2G (acorde a requerimientos)
constexpr int       TIEMPO_MS_MIN_BOOST                     = 200;          // 0.2 segundos en milisegundos (acorde a requerimientos)
constexpr uint32_t  CONEXION_GSE_TIMEOUT_MILLIS             = 1000;         // Solo debemos esperar un minuto
constexpr uint8_t   CANT_REINTENTOS_TMOUT                   = 2;            // Cantidad de reintentos que hacemos para conectarnos a la gse
constexpr float     ALTURA_M_MAX                            = 1000;         // TODO: chequear ALTURA_M_MAX. Igual nos importa realmente este dato?
constexpr uint32_t  GPS_TIMEOUT_MILLIS                      = 1000*5;
constexpr uint32_t  TIEMPO_MILLIS_ESPERA_WARMUP_MPU         = 1000*60*5;
constexpr int       DIFF_ALTURA_M_APOGEO_CAIDA              = 10;
constexpr int       ALTITUD_DESPLIEGUE_PCAIDAS_PPAL         = 250;



namespace Cohete {

    namespace Timers
    {
        static TimerHandle_t xTimerRecalibrarMPU;
        static volatile bool flag_recalibrarMPU_disparado = false; // TODO: huelo una pequeña condicion de carrera, que en este caso zafa.

        // typedef void (* TimerCallbackFunction_t)( TimerHandle_t xTimer );
        void calibrar_mpu_callback(TimerHandle_t xTimer) {
            xTaskCreate(
    [](void* pvParameters) {
                    ESP_LOGI(TAG_BASE, "Temporizador xTimerRecalibrarMPU disparado!");
                    ESP_LOGI(TAG_BASE, "Calibrando MPU6050.");

                    const mMPU6050::CalibrationStatus res = Sensors::getMPU6050().calibrar();

                    if (res == mMPU6050::CalibrationStatus::Ok) {
                        flag_recalibrarMPU_disparado = true;
                        ESP_LOGI(TAG_BASE, "Calibración de MPU6050 exitosa.");
                    }

                    vTaskDelete(NULL); // para auto-borrarse
                },
                "MPU_Calibrar_Task", // Name of the task for debugging
                4096,                // Stack size in words (or bytes in ESP-IDF)
                NULL,                // Parameter passed into the task (pvParameters)
                5,                   // Task priority (adjust as needed)
                NULL                 // Task handle (optional, pass &handle if needed)
            );
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    namespace Eventos {
        // val0 --> valor de estudio
        // val1 --> valor objetivo
        // delta --> margen
        /*inline bool aproxima(const float val0, const float val1, const float delta) {
            return fabsf(val0 - val1) <= delta; // implementación que aprovecha la FPU de la ESP32
            // return (val0 <= val1 + delta && val0 >= val1 - delta);
        }
        */
        // lo mismo que decir: es val0 menor por 5 unidades a val1?
        // inline bool menor_delta_que(const int val0, const int val1, const float delta) {
        //     return (val0 <= val1 + delta);
        // }

        inline bool entorno (float X, float centro, float radio){
            return fabsf(X - centro) <= radio;
        }
        // TODO: SUPONGO QUE VAMOS A QUERER OVERRIDEAR ESTA CONDICION, EN CASO DE QUE LAS CONDICIONES NO CUMPLAN, LANZAR IGUAL EL COHETE.
        bool gps_es_preciso(const data_all_t* datos_sensores) {
            static uint8_t ticks_cumple_condiciones = 0;

            const bool cumple =
                datos_sensores->gps_nro_satelites >= 5       // Mínimo 4 para 3D, 5 o 6 es más seguro
                && datos_sensores->gps_is_valid
                && datos_sensores->gps_hdop <= 2.0;              // Dilución de precisión menor o igual a 2.0 es un buen valor

            if (cumple && (ticks_cumple_condiciones < 255) ) {
                ticks_cumple_condiciones++;
            }
            else {
                ticks_cumple_condiciones = 0; // se reinicia el contador si UNA sola vez no se cumple la condicion.
            }

            // queremos que se cumpla la condicion 50 ticks de corrido
            if (ticks_cumple_condiciones > 50) {
                return true;
            }

            return false;
        }

        bool en_codiciones_para_volar(data_all_t* dat_ssr) {
            // Verificación geométrica de inclinación en rampa
            bool estaListo = entorno(dat_ssr->angulo_respecto_z_deg, 0.0f, 5.0f);
            if (!estaListo) return false;

            // Verificación de borrado de memoria Flash
            if (Cohete::SYSTEM.flags.gse_conectado) {
                // Si hay enlace GSE, dependemos de la bandera de confirmación
                estaListo = Cohete::SYSTEM.flags.flash_log_borrado;
            } 
            else {
                // Si no hay GSE pero la memoria no está borrada aún:
                if (!Cohete::SYSTEM.flags.flash_log_borrado) {
                    
                    // Dispara la orden SOLO SI no fue disparada previamente
                    if (!Cohete::SYSTEM.accion.borrar_log) {
                        ESP_LOGW(TAG_BASE, "Autoborrado de Flash solicitado por MdE en rampa...");
                        Cohete::SYSTEM.accion.borrar_log = true;
                        
                        // Notifica a la tarea de Flash para que despierte ya mismo
                        if (Cohete::SYSTEM.procesos.xTaskFlashHandle != NULL) {
                            xTaskNotifyGive(SYSTEM.procesos.xTaskFlashHandle);
                        }
                    }
                    return false; // Aún no esta lista, espera que vTaskFlash termine
                }
                estaListo = true;
            }

            // Sí se calibró, estamos listos
            estaListo = Timers::flag_recalibrarMPU_disparado;
            if (!estaListo) return false;

            // Si no se saltea el GPS y tampoco es preciso, entonces no estamos listos
            if (!Cohete::SYSTEM.gse_configs.gps_override_skip && !Cohete::SYSTEM.flags.gps_preciso) {
                return false;
            }

            estaListo = true;


            return estaListo;
        }

        bool hay_boost(data_all_t *datos_sensores) { 
            // Memoria para recordar que la condición de empuje inicial se cumplió
            static bool empuje_sostenido_confirmado = false;

            // Evaluar aceleración de ignición
            if (datos_sensores->aceleracion_z_m_s2 >= UMBRAL_ACEL_BOOST) { 
                if (SYSTEM.timestamp_millis_inicio_pico_g == 0) { 
                    SYSTEM.timestamp_millis_inicio_pico_g = millis(); 
                } 
                else if ((millis() - SYSTEM.timestamp_millis_inicio_pico_g) >= TIEMPO_MS_MIN_BOOST) {
                    // Ya tuvimos 200ms sostenidos. El motor encendió. Guardamos el estado.
                    empuje_sostenido_confirmado = true;
                }
            } else {
                // CRÍTICO: Solo reiniciamos el cronómetro si el empuje NO había sido confirmado
                // Esto evita abortar el despegue si la IMU lee < 2g por ruido o fin de combustión
                if (!empuje_sostenido_confirmado) {
                    SYSTEM.timestamp_millis_inicio_pico_g = 0;
                }
            }

            // Condición principal: Motor encendido Y superamos los 4 metros (salida de rampa)
            if (empuje_sostenido_confirmado && datos_sensores->altitud_filtrada_m > (SYSTEM.ctx_fisico.altitud_m_pad + 4.0f)) {
                empuje_sostenido_confirmado = false; // Limpiamos la bandera para un futuro
                return true; 
            }

            // Condición de seguridad (Fallback / Redundancia)
            // Si por ruido extremo o falla de la IMU jamás leímos 2G constantes, pero
            // de repente estamos 10 metros por encima de la rampa, estamos volando sí o sí.
            if (datos_sensores->altitud_filtrada_m > (SYSTEM.ctx_fisico.altitud_m_pad + 10.0f)) {
                return true;
            }

            return false;
        }

    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * @brief Dispara un timer de 5 min para calentar la IMU
     */
    void f_st_init(data_all_t* datos_sensores, uint32_t ms_en_estado) {
        ESP_LOGI(TAG_BASE, "INIT");

        //TODO: LÓGICA DE LECTURA DE FLASH = X
        // Note: Delegado en main, máxima prioridad 


        // Esperar 5 minutos para que la mpu entre en calor, luego calibrarla.
        Timers::xTimerRecalibrarMPU =
            xTimerCreate(
                "Recalibrar",
                pdMS_TO_TICKS(TIEMPO_MILLIS_ESPERA_WARMUP_MPU),
                pdFALSE, // one shot timer
                nullptr,
                Timers::calibrar_mpu_callback
            );

        if(Timers::xTimerRecalibrarMPU != NULL ) {
            /* Iniciamos el temporizador con un tiempo de bloqueo (block time) de 0 */
            ESP_LOGI(TAG_BASE, " -> [INIT] Temporizador xTimerRecalibrarMPU creado. Se disparará en 5 minutos"); // TODO: mejorar sistema de logging
            xTimerStart(Timers::xTimerRecalibrarMPU, 0 );
            if(EnlaceGSE::enviarMensaje("[INIT] Temporizador xTimerRecalibrarMPU creado. Se disparará en 5 minutos")){
                ESP_LOGI(TAG_BASE, "Mensaje enviado a enlaceGSE");
            }
            else{
                ESP_LOGE(TAG_BASE, "No se pudo enviar msg a enlaceGSE");
            }
        }

        // Se debería llamar a la función _lora.c_connect_to_GSE() por medio de GSE o EnlaceGSE, 
        //_lora.c_connect_to_GSE(); EMITE UN PING
        
        ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Entrando a: Espera conexión con GSE...");
        SYSTEM.flags.gse_conectado = false; 
        SYSTEM._estado = ST_ESPERA_CONEXION_GSE;
        // transicionar_hacia(ST_ESPERA_CONEXION_GSE);
    }

    void f_st_espera_conexion_gse(data_all_t* datos_sensores, uint32_t ms_en_estado) {
        //  Condición de éxito: ¿Se conectó por LoRa?
        if (SYSTEM.flags.gse_conectado) {
            ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Nos conectamos exitosamente.");
            EnlaceGSE::enviarMensaje("Conexion establecida. Buscando GPS.");
            SYSTEM._estado = ST_ESPERA_GPS_PRECISO;
            return;
        }

        // Condición de Timeout: Evaluado directamente contra el argumento
        if (ms_en_estado >= CONEXION_GSE_TIMEOUT_MILLIS) {
            ESP_LOGE(TAG_BASE, "Timeout de conexión con GSE. Continuando vuelo...");
            SYSTEM._estado = ST_ESPERA_GPS_PRECISO;
        }
    }
    /*
    void f_st_espera_conexion_gse(data_all_t* datos_sensores, uint32_t ms_en_estado) {
        static uint64_t timestamp_millis_inicio_timeout = millis();
        ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Esperando conexión con GSE...");
        // Lógica:
        // 1. Intentar recibir configuración por LoRa (GSE).
        // 2. Si llegan los datos, configurar y pasar a ST_ESPERA_DESPEGUE.
        // 3. Si (millis() - t_inicio_busqueda > TIMEOUT_LORA), entonces:
        //    COHETE.vuelo_en_silencio_radio = true;
        //    Transición forzada a ST_ESPERA_DESPEGUE (el despegue es prioridad).
        // if (entrando_a_estado()) {
        //     timestamp_millis_inicio_timeout = millis();
        //     ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Esperando conexión con GSE...");
        // }
        
        if (GSE::estado_conexion_gse() == ROCKET_CONNECTED) {
            ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Nos conectamos a la GSE.");
            //transicionar_hacia(ST_ESPERA_GPS_PRECISO);
            SYSTEM._estado = ST_ESPERA_GPS_PRECISO;
        }
            // EN PARALELO SE DEBE EJECUTAR EL LORA (task), y escuchar las respuestas del GSE, [ademas del comando]
            // Revisar que haya llegado el PONG PARA TRANSICIONAR A OTRO ESTADO
        // TODO: Qué hacemos mientras cohete espera conexión con GSE ?
        // TODO: AGREGAR LÓGICA DE REINTENTOS, AL MENOS 2.
        if ((millis() - timestamp_millis_inicio_timeout) >= CONEXION_GSE_TIMEOUT_MILLIS) { // TODO: Revisar si el timeout del gse es conveniente
            // transicion_error(ERR_TIMEOUT_CONEXION_GSE, datos_sensores); // TODO: empiezo a considerar que la función "transicion_error" genera un nivel de indirección innecesario. Se podría poner la lógica del error acá mismo.
            ESP_LOGE(TAG_BASE, "Timeout de conexión con GSE.");
            // matamos el proceso GSE asi no nos gasta recursos del cohete, o bajamos su frecuencia.
            // vTaskSuspend(SYSTEM.procesos.xTaskLoraHandle);
            // NUNCA MATAR, ENVIAR CONTINUAMENTE. 
            // TODO: SÓLO SI EL HARDWARE NO SE PUDO INICIALIZAR EL MÓDULO, NO DEBERIA HABERSE CREADO LA TAREA (no implementado)

            SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false;
            ESP_LOGI(TAG_BASE, "Suspendido el Task Lora, ya que no nos comunicaremos con la GSE.");
            // continuamos con la siguiente etapa.
            //transicionar_hacia(ST_ESPERA_GPS_PRECISO);
            SYSTEM._estado = ST_ESPERA_GPS_PRECISO;
        }
    }
    */

void f_st_espera_gps_preciso(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    // Timeout
    if (ms_en_estado >= GPS_TIMEOUT_MILLIS) {
        ESP_LOGE(TAG_BASE, "Timeout de GPS. Avanzando a espera de ignición.");
        EnlaceGSE::enviarError("[GPS] Timeout ");
        Cohete::SYSTEM.flags.gps_preciso = false;
        SYSTEM._estado = ST_ESPERA_IGNICION;
        return;
    }
    Cohete::SYSTEM.flags.gps_preciso = Eventos::gps_es_preciso(datos_sensores);
    // Condición de éxito o bypass de GSE
    if ( Cohete::SYSTEM.flags.gps_preciso || SYSTEM.gse_configs.gps_override_skip) {
        ESP_LOGI(TAG_BASE, " -> [GPS] Precisión asegurada.");
        EnlaceGSE::enviarMensaje(
            (Cohete::SYSTEM.gse_configs.gps_override_skip)?
                "[GPS] SKIP busqueda":
                "[GPS] Precisión asegurada."
            );
        Cohete::SYSTEM._estado = ST_ESPERA_IGNICION;
        return;
    }

    // LOG
    if (ms_en_estado% 200 == 0){
        ESP_LOGI(TAG_BASE, " -> [GPS] Esperando satélites. Visibles: %d", datos_sensores->gps_nro_satelites);
    }

}

    void f_st_espera_ignicion(data_all_t* datos_sensores, uint32_t ms_en_estado) {
        // Aquí el cohete está pasivo en la rampa. Ignición es externa.
        // Lógica del filtro anti-zarandeo:
        // 1. Transformar aceleración a vector inercial.
        // 2. if (aceleracion_vertical >= 2.0g) {
        //      iniciar temporizador (timestamp_inicio_pico_g)
        //    }
        // 3. if (tiempo_con_2g >= 150ms AND delta_altura > 4.0m) {
        //      Transición a ST_PROPULSION.
        //    }

        if(Eventos::en_codiciones_para_volar(datos_sensores)) {
            // if led no encendido: encenderlo para señalizar que ya podemos volar.
            if (ms_en_estado >= 3000) { // hacemos un beep cada 3 segundos
                ESP_LOGI(TAG_BASE, " -> [ESPERA IGNICION] Cohete en condiciones para volar!");
            }

            if (Eventos::hay_boost(datos_sensores)) {
                // SYSTEM.timestamp_millis_inicio_pico_g = micros();
                EnlaceGSE::enviarMensaje("Depegue Autorizado");
                Cohete::SYSTEM._estado = ST_BOOST;
            }
        }
        else if (Eventos::hay_boost(datos_sensores)) { // detectamos boost a pesar de no estar en condiciones para volar, KEEEEE!!!!!!
            // transicion_error(ERR_DESPEGUE_PROHIBIDO, datos_sensores);
            ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_DESPEGUE_PROHIBIDO");
            // L: Qué hacemos si realmente detectamos un despegue, pero el cohete no estaba en condiciones de volar? Pienso que: ya que esta en vuelo, mucho no podemos hacer al respecto, hay que continuar con lo que se tiene. Ver qué hacemos a partir de ahí.
            // J: TOTALMENTE. Que sea lo que Dios quiera
            
            EnlaceGSE::enviarError("Despegue no autorizado");
            Cohete::SYSTEM._estado = ST_BOOST;
        }
    }

    void f_st_boost(data_all_t* datos_sensores, uint32_t ms_en_estado) {
        
        // ---------------------------------------------------------
        // 1. SEGURIDAD MÁXIMA: Detección de trayectoria anómala (Misil)
        // ---------------------------------------------------------
        // Si la inclinación supera los 30 grados, perdimos la verticalidad.
        // Aplicamos debounce para evitar disparos por ruido de la IMU.
        // Hay un tick cada 5ms -> 15 ms de debouce
        static uint8_t ticks_inclinacion_peligrosa = 0;
        
        if (datos_sensores->angulo_respecto_z_deg > 30.0f ) { 
            if (ticks_inclinacion_peligrosa < 255) ticks_inclinacion_peligrosa++;
        } else {
            ticks_inclinacion_peligrosa = 0;
        }

        if (ticks_inclinacion_peligrosa >= 3) {
            EnlaceGSE::enviarError("[ABORT] INCLINACION CRITICA. FORZANDO ABORTO.");
            
            // Disparar pirotécnicos para desestabilizar el cohete
            Actuators::getPyroDrogue().armar();
            Actuators::getPyroDrogue().disparar();
            
            SYSTEM._estado = ST_CAIDA_CATASTROFICA; // O un estado específico de ABORTO
            return; 
        }

        // ---------------------------------------------------------
        // 2. DETECCIÓN DE BURN-OUT (Transición a Balística)
        // ---------------------------------------------------------
        // Ignoramos el primer tramo (ej. 5000ms) donde la curva de empuje 
        // podría tener fluctuaciones iniciales propias del motor sólido.
        if (ms_en_estado > 1000) {
            
            // Si la aceleración Z (sin gravedad) cae por debajo de 0, 
            // significa que el empuje es menor que el Drag. El motor se apagó.
            if (datos_sensores->aceleracion_z_m_s2 <= 0.0f) {
                
                // Verificamos que sigamos subiendo a buena velocidad como doble chequeo
                if (datos_sensores->vel_z_filtrada_m_s > 5.0f) {
                    EnlaceGSE::enviarMensaje("[BOOST] MOTOR APAGADO. MODO BALISTICO.");
                    
                    // Actualizamos la masa antes de entrar a balística
                    SYSTEM.ctx_fisico.masa_g_cohete -= SYSTEM.ctx_fisico.masa_g_combustible;
                    
                    SYSTEM._estado = ST_FASE_BALISTICA;
                    return;
                }
            }
        }
    }

void f_st_fase_balistica(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    // ---------------------------------------------------------
    // 0. RESET DE VARIABLES ESTÁTICAS AL ENTRAR AL ESTADO
    // ---------------------------------------------------------

    static uint8_t ticks_apogeo_confirmado = 0;
    if (ms_en_estado == 0) {
        ticks_apogeo_confirmado = 0;
    }
    // ---------------------------------------------------------
    // 1. ACTUALIZACIÓN DE ALTURA MÁXIMA
	// ---------------------------------------------------------
    if ((datos_sensores->vel_z_filtrada_m_s > -0.2f) &&
        (datos_sensores->altitud_filtrada_m > SYSTEM.ctx_fisico.altura_m_max_historica)) 
    {
        SYSTEM.ctx_fisico.altura_m_max_historica = datos_sensores->altitud_filtrada_m;
    }
    
    // ---------------------------------------------------------
    // 2. GATES DE DETECCIÓN DE APOGEO
    // ---------------------------------------------------------  
  	const bool vel_apogeo = (datos_sensores->vel_z_filtrada_m_s <= -0.3f);
    const bool caida_confirmada = (SYSTEM.ctx_fisico.altura_m_max_historica - datos_sensores->altitud_filtrada_m) >= DIFF_ALTURA_M_APOGEO_CAIDA; 
    const bool g_gate_valido = (datos_sensores->aceleracion_z_m_s2 > -5.0f && datos_sensores->aceleracion_z_m_s2 < 2.0f);

    if ((vel_apogeo && g_gate_valido) || caida_confirmada) {
        if (ticks_apogeo_confirmado < 255) ticks_apogeo_confirmado++;
    } else {
        ticks_apogeo_confirmado = 0; 
    }

    const bool apogeo_alcanzado = (ticks_apogeo_confirmado >= 5);

    // ---------------------------------------------------------
    // 3. SECUENCIA DE APOGEO Y DISPARO
    // ---------------------------------------------------------
    if (apogeo_alcanzado) {
        Actuators::getServo().sendAngulo(0.0f); // Retraer Airbrake
		// Sub-máquina de estados 
        // A) Ningún pirotécnico ha sido iniciado todavía
        if (!SYSTEM.flags.drogue_disparado && !SYSTEM.flags.paracaidas_principal_disparado) {
            
            // INTENTO 1: Drogue, verifica que el ignitor esté sano
            if (Actuators::getPyroDrogue().tieneContinuidad()) {
                ESP_LOGI(TAG_BASE, "[APOGEO] Disparando Drogue...");
                Actuators::getPyroDrogue().armar();
                Actuators::getPyroDrogue().disparar();
                
                SYSTEM.flags.drogue_disparado = true;
                SYSTEM.timestamp_micros_apertura_drogue = micros(); 
            } 
            // FALLBACK: Drogue sin continuidad -> Disparar Principal directo
            else if (Actuators::getPyroPpal().tieneContinuidad()) {
                ESP_LOGW(TAG_BASE, "[APOGEO] Drogue SIN continuidad. Disparando Principal (FALLBACK)...");
                Actuators::getPyroPpal().armar();
                Actuators::getPyroPpal().disparar();
                
                SYSTEM.flags.paracaidas_principal_disparado=true; 
                SYSTEM.timestamp_micros_apertura_drogue = micros(); // Reutilizamos temporizador de pulso
            } 
            // FAIL CRÍTICO: Ningún ignitor funciona
            else {
                ESP_LOGE(TAG_BASE, "[CRÍTICO] ¡Sin continuidad en NINGÚN pirotécnico!");
                SYSTEM._estado = ST_CAIDA_CATASTROFICA;
                return;
            }
        } 
        // B) El disparo ya se ordenó. Mantenemos el MOSFET 100ms activo (Asíncrono)
        else {
            if ((micros() - SYSTEM.timestamp_micros_apertura_drogue) >= 100000UL) {
                
                if (SYSTEM.flags.drogue_disparado) {
                    Actuators::getPyroDrogue().desarmar();
                    ESP_LOGI(TAG_BASE, "[APOGEO] Drogue ignisecus OK. Transición a ST_DROGUE_DESPLEGADO");
                    SYSTEM._estado = ST_DROGUE_DESPLEGADO;
                } else {
                    Actuators::getPyroPpal().desarmar();
                    ESP_LOGI(TAG_BASE, "[APOGEO] Principal ignisecus OK. Transición a ST_PCAIDAS_PPAL_DESPLEGADO");
                    SYSTEM._estado = ST_PCAIDAS_PPAL_DESPLEGADO;
                }
                return;
            }
        }
    } 
    // 4. CONTROL DE AIRBRAKES EN ASCENSO
    else {
        float angulo_airbrake = 0.0f;
        if (datos_sensores->vel_z_filtrada_m_s >= 15.0f) {
            // angulo_airbrake = MPC_Controller::calcular_accion(...);
        }
        Actuators::getServo().sendAngulo(angulo_airbrake);
    }
}


void f_st_drogue_desplegado(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    static uint8_t ticks_despliegue_ppal = 0;
    static bool disparo_ppal_iniciado = false;
    static uint32_t t_inicio_pulso_ppal = 0;

    if (ms_en_estado == 0) {
        ticks_despliegue_ppal = 0;
        disparo_ppal_iniciado = false;
    }

    // ---------------------------------------------------------
    // 1. VERIFICACIÓN DE SALUD DEL DROGUE (A los 3 segundos)
    // ---------------------------------------------------------
    // Si caemos demasiado rápido (ej. vel < -35 m/s), el drogue falló o se desprendió.
    if (ms_en_estado >= 3000 && !disparo_ppal_iniciado) {
        if (datos_sensores->vel_z_filtrada_m_s < -35.0f) {
            ESP_LOGE(TAG_BASE, "[ALERTA] Caída excesivamente rápida (%.1f m/s). ¡Drogue falló!", datos_sensores->vel_z_filtrada_m_s);
            // Adelantamos la apertura del principal por emergencia
            ticks_despliegue_ppal = 10; 
        }
    }

    // ---------------------------------------------------------
    // 2. CONDICIÓN NOMINAL: ALTITUD DE DESPLIEGUE DEL PRINCIPAL
    // ---------------------------------------------------------
    if (SYSTEM.ctx_fisico.altitud_m_relativa_al_pad <= ALTITUD_DESPLIEGUE_PCAIDAS_PPAL && datos_sensores->vel_z_filtrada_m_s < 0.0f) {
        if (ticks_despliegue_ppal < 255) ticks_despliegue_ppal++;
    } else if (ticks_despliegue_ppal < 10) {
        ticks_despliegue_ppal = 0;
    }

    // ---------------------------------------------------------
    // 3. SECUENCIA DE DISPARO DEL PARACAÍDAS PRINCIPAL
    // ---------------------------------------------------------
    if (ticks_despliegue_ppal >= 10) {
        
        if (!disparo_ppal_iniciado) {
            if (Actuators::getPyroPpal().tieneContinuidad()) {
                ESP_LOGI(TAG_BASE, "[DESCENSO] Cota alcanzada. Disparando Paracaídas Principal...");
                Actuators::getPyroPpal().armar();
                Actuators::getPyroPpal().disparar();
                
                SYSTEM.flags.paracaidas_principal_disparado = true;
                disparo_ppal_iniciado = true;
                t_inicio_pulso_ppal = micros();
            } else {
                ESP_LOGE(TAG_BASE, "[CRÍTICO] Sin continuidad en Pyro Principal. Transición a Caída Catastrófica.");
                SYSTEM._estado = ST_CAIDA_CATASTROFICA;
                return;
            }
        } 
        else {
            // Mantenemos el pulso de 100 ms antes de cortar corriente y transicionar
            if ((micros() - t_inicio_pulso_ppal) >= 100000UL) {
                Actuators::getPyroPpal().desarmar();
                ESP_LOGI(TAG_BASE, "[DESCENSO] Principal desplegado con éxito.");
                SYSTEM._estado = ST_PCAIDAS_PPAL_DESPLEGADO;
                return;
            }
        }
    }
}


void f_st_pcaidas_ppal_desplegado(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    static uint16_t ticks_aterrizado = 0;

    if (ms_en_estado == 0) {
        ticks_aterrizado = 0;
        ESP_LOGI(TAG_BASE, "[DESCENSO] Descendiendo bajo Paracaídas Principal...");
    }

    // ---------------------------------------------------------
    // 1. CRITERIO DE ATERRIZAJE (TOUCHDOWN)
    // ---------------------------------------------------------
    // Condición A: Velocidad vertical prácticamente nula (-0.4 m/s < vel < 0.4 m/s)
    const bool velocidad_nula = (datos_sensores->vel_z_filtrada_m_s > -0.4f && datos_sensores->vel_z_filtrada_m_s < 0.4f);
    
    // Condición B: Aceleración estática (sin sacudidas del paracaídas ni viento fuerte)
    const bool aceleracion_estatica = (datos_sensores->aceleracion_z_m_s2 > -1.5f && datos_sensores->aceleracion_z_m_s2 < 1.5f);

    if (velocidad_nula && aceleracion_estatica) {
        if (ticks_aterrizado < 65535) ticks_aterrizado++;
    } else {
        ticks_aterrizado = 0; // Si el cohete arrastra en el suelo por el viento, reseteamos
    }

    // ---------------------------------------------------------
    // 2. CONFIRMACIÓN DE ATERRIZAJE (Ej. ~3 segundos sostenidos)
    // ---------------------------------------------------------
    // Asumiendo refresco de MdE a ~50-100Hz, 200 ticks son aprox. 2 a 4 segundos de reposo absoluto
    if (ticks_aterrizado >= 200 || ms_en_estado >= 300000) { // Fallback de tiempo: 5 minutos de descenso máximo
        
        ESP_LOGI(TAG_BASE, "==================================================");
        ESP_LOGI(TAG_BASE, "¡¡TOUCHDOWN CONFIRMADO!! EL COHETE HA ATERRIZADO.");
        ESP_LOGI(TAG_BASE, "Altitud Final Pad: %.2f m", SYSTEM.ctx_fisico.altitud_m_relativa_al_pad);
        ESP_LOGI(TAG_BASE, "==================================================");

        // Guardar coordenadas finales GPS para rescate
        if (datos_sensores->gps_is_valid) {
            char msg[128] = {};
            sprintf(msg, "[RESCATE] GPS Lat: %f, Lon: %f", datos_sensores->gps_latitud, datos_sensores->gps_longitud);
            EnlaceGSE::enviarMensaje(msg);
        }

        // Transicionar al estado final de reposo/recuperación
        SYSTEM._estado = ST_ATERRIZADO;
    }
}

    // void f_st_descenso_controlado_drogue(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    //     // Bajando con Drogue.
    //     // Lógica:
    //     // if (altura_actual_filtrada <= 250.0m) { // ¡Cuidado de chequear contra cota_suelo_rampa!
    //     //      Ignición pirotécnica Paracaídas Principal.
    //     //      // ESP_LOGI(TAG_STATE_MACHINE, " -> [NOMINAL] Paracaídas Principal desplegado.");
    //     //      Transición a ST_ATERRIZAJE (o estado intermedio de espera).
    //     // }
    // }

    // void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    //     ESP_LOGI(TAG_BASE, " -> [EMERGENCIA] Drogue fallido. Disparando Principal de inmediato!");
    //     // Lógica:
    //     // 1. Disparo inmediato del paracaídas principal.
    //     // 2. Transición a ST_ATERRIZAJE (esperando el suelo).
    // }

void f_st_caida_catastrofica(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    
    // Solo ejecutamos las acciones críticas al entrar al estado
    if (ms_en_estado == 0) {
        ESP_LOGE(TAG_BASE, "==================================================");
        ESP_LOGE(TAG_BASE, " -> [FATAL] CAÍDA CATASTRÓFICA DETECTADA.");
        ESP_LOGE(TAG_BASE, " -> INICIANDO PROTOCOLO DE EMERGENCIA POST-MORTEM.");
        ESP_LOGE(TAG_BASE, "==================================================");

        // 1. Activar bandera global de pánico  cargar datos
        SYSTEM.flags.emergencia_fatal = true;
        SYSTEM.datos_actuales.gps_latitud   = datos_sensores->gps_latitud;
        SYSTEM.datos_actuales.gps_longitud  = datos_sensores->gps_longitud;

        // 2. Medida desesperada: Forzar disparo del principal si tiene continuidad
        // Si ya se disparó antes o está roto, esto fallará de forma segura.
        // Si por algún bug de software no se había abierto, esto podría salvar el fuselaje.
        if (Actuators::getPyroPpal().tieneContinuidad()) {
            Actuators::getPyroPpal().armar();
            Actuators::getPyroPpal().disparar();
        }

        // 3. Ordenar el volcado de RAM a Flash Interna (LittleFS/SPIFFS)
        SYSTEM.accion.volcar_ram_a_flash = true;
        if (SYSTEM.procesos.xTaskFlashHandle != NULL) {
            xTaskNotifyGive(SYSTEM.procesos.xTaskFlashHandle);
        }

        // 4. Despertar a la tarea LoRa INMEDIATAMENTE para iniciar el SOS
        if (SYSTEM.procesos.xTaskLoraHandle != NULL) {
            xTaskNotifyGive(Cohete::SYSTEM.procesos.xTaskLoraHandle);
        }
    }

    // Lógica continua (opcional):
    // Como es terminal, la MdE puede dejar de procesar matemática pesada (como el MPC)
    // para cederle todo el tiempo de CPU y energía a las tareas de LoRa y Flash.
    // Retraemos el servo para evitar que se rompa en el impacto.
    Actuators::getServo().sendAngulo(0.0f);
}

void f_st_aterrizado(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    // Lógica:
    

     // Solo ejecutamos las acciones críticas al entrar al estado
    if (ms_en_estado == 0) {
        ESP_LOGE(TAG_BASE, "==================================================");
        ESP_LOGE(TAG_BASE, " -> [EXITO] ATERRIZAJE CORRECTO.");
        ESP_LOGE(TAG_BASE, "==================================================");

        // 1. Activar bandera global de pánico  cargar datos
        SYSTEM.flags.emergencia_fatal = true;
        SYSTEM.datos_actuales.gps_latitud   = datos_sensores->gps_latitud;
        SYSTEM.datos_actuales.gps_longitud  = datos_sensores->gps_longitud;


        // 2. Ordenar el volcado de RAM a Flash Interna (LittleFS/SPIFFS)
        SYSTEM.accion.volcar_ram_a_flash = true;
        if (SYSTEM.procesos.xTaskFlashHandle != NULL) {
            xTaskNotifyGive(SYSTEM.procesos.xTaskFlashHandle);
        }

        // 3. Despertar a la tarea LoRa INMEDIATAMENTE para iniciar el SOS
        if (SYSTEM.procesos.xTaskLoraHandle != NULL) {
            xTaskNotifyGive(Cohete::SYSTEM.procesos.xTaskLoraHandle);
        }
    }
    
    // 2. Detener logs de alta frecuencia para salvar batería.


    if(ms_en_estado % 500 ==0){
        // Suponiendo que los sensores funcionan... MEJOR NO xd
        SYSTEM.datos_actuales.gps_longitud  = datos_sensores->gps_longitud;
        SYSTEM.datos_actuales.gps_latitud   = datos_sensores->gps_latitud;
        buzzer.playSuccess();
    }
    // 3. Cerrar archivos en la SD (flush y close).
    // 4. Emitir un "beeeeep beeeeep" continuo y transmitir coordenadas Lat/Lon por LoRa cada X segundos.
}

}
/**
 * Si el drogue no tiene continuidad, abrir el paracaidas
 * en el descenso
 * 100m/S menor use usa
 */
