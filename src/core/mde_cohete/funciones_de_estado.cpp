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
constexpr float     ACEL_M_S2_UMBRAL_BOOST                  = 2 * A_GRAV;   // 2G (acorde a requerimientos)
constexpr int       TIEMPO_MS_MIN_BOOST                     = 200;          // 0.2 segundos en milisegundos (acorde a requerimientos)
constexpr uint32_t  CONEXION_GSE_TIMEOUT_MILLIS             = 1000;         // Solo debemos esperar un minuto
constexpr uint8_t   CANT_REINTENTOS_TMOUT                   = 2;            // Cantidad de reintentos que hacemos para conectarnos a la gse
constexpr float     ALTURA_M_MAX                            = 1000;         // TODO: chequear ALTURA_M_MAX. Igual nos importa realmente este dato?
constexpr float     ALTURA_M_MIN                            = 500;
constexpr uint32_t  GPS_TIMEOUT_MILLIS                      = 1000*5;
constexpr uint32_t  TIEMPO_MILLIS_ESPERA_WARMUP_MPU         = 1000*20;
constexpr int       DIFF_ALTURA_M_APOGEO_CAIDA              = 10;
constexpr int       ALTITUD_DESPLIEGUE_PCAIDAS_PPAL         = 250;


namespace Cohete {
    uint32_t t_time_ms() {
        return CONFIG_RESTAURACION.t_time_ms_mision_anterior + millis();
    }
    uint64_t t_time_us() {
        return CONFIG_RESTAURACION.t_time_us_mision_anterior + micros();
    }

    namespace Timers
    {
        static TimerHandle_t xTimerRecalibrarMPU;
        static volatile bool flag_recalibrarMPU_disparado = false; // TODO: huelo una pequeña condicion de carrera, que en este caso zafa.

        void calibrar_mpu_callback(TimerHandle_t xTimer) {
            // typedef void (* TimerCallbackFunction_t)( TimerHandle_t xTimer );
            auto calibracion_mpu = [](void* pvParameters) {
                ESP_LOGI(TAG_BASE, "Temporizador xTimerRecalibrarMPU disparado!");
                ESP_LOGI(TAG_BASE, "Calibrando MPU6050.");

                const mMPU6050::CalibrationStatus res = Sensors::getMPU6050().calibrar();

                if (res == mMPU6050::CalibrationStatus::Ok) {
                    flag_recalibrarMPU_disparado = true;
                    ESP_LOGI(TAG_BASE, "Calibración de MPU6050 exitosa.");
                }

                vTaskDelete(NULL); // para auto-borrarse
            };
            xTaskCreatePinnedToCore(calibracion_mpu, "MPU_Calibrar_Task", 4096,NULL, ConfigInit::TASK_PRIORITY_COMMON, NULL,1);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    namespace Eventos {
        // lo mismo que decir: es val0 menor por 5 unidades a val1?
        // inline bool menor_delta_que(const int val0, const int val1, const float delta) {
        //     return (val0 <= val1 + delta);
        // }

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

#define TOLERANCIA_RUIDO_MS            50     // Permite caídas de hasta 50ms por vibración antes de resetear
#define ALTITUD_MIN_SALIDA_RAMPA_M     4.0f   // Altura para confirmar salida de rampa
#define ALTITUD_FALLBACK_M             12.0f  // Altura de redundancia
#define VELOCIDAD_VERT_MIN_FALLBACK    5.0f   // ~5 m/s para validar que realmente subimos (evita viento en rampa)
        bool hay_boost_garantizado(const data_all_t *datos_sensores) {
            if (datos_sensores == NULL) {
                return false;
            }
            if (!isfinite(datos_sensores->aceleracion_vertical_m_s2_mpu) ||
                !isfinite(datos_sensores->altitud_asl_filtrada_m_bmp) ||
                !isfinite(datos_sensores->velocidad_vertical_filtrada_m_s)) {
                // Datos corruptos por ruido I2C, NaN o infinito. Abortar evaluación este ciclo.
                return false;
                }

            const uint32_t ahora_ms = millis();
            static uint32_t timestamp_ultima_acel_valida = 0;
            static uint32_t timestamp_millis_inicio_pico_g = 0;

            // Serial.printf("aceleracion_vertical_m_s2_mpu=%f\n", datos_sensores->aceleracion_vertical_m_s2_mpu);
            // 1. Evaluación de Aceleración con Ventana de Tolerancia (Debounce)
            if (datos_sensores->aceleracion_vertical_m_s2_mpu >= ACEL_M_S2_UMBRAL_BOOST) {
                if (timestamp_millis_inicio_pico_g == 0) {
                    EnlaceGSE::enviarMensaje("Sospecha de BOOST");
                    timestamp_millis_inicio_pico_g = ahora_ms;
                }
                timestamp_ultima_acel_valida = ahora_ms;
            }
            // Si dejamos de tener pico de Gs, pero ya habiamos registrado un inicio de pico de Gs anteriormente...
            else if (timestamp_millis_inicio_pico_g != 0) {
                // Caída de aceleración durante el empuje: ¿Es vibración (<50ms) o apagado de motor (>50ms)?
                if ((ahora_ms - timestamp_ultima_acel_valida) > TOLERANCIA_RUIDO_MS) {
                    // Se limpian AMBOS temporizadores para no arruinar la tolerancia de futuros ciclos
                    timestamp_millis_inicio_pico_g = 0;
                    timestamp_ultima_acel_valida = 0;
                }
                EnlaceGSE::enviarMensaje("Falsa detección de BOOST");
                ESP_LOGI(TAG_BASE, "Falsa detección de BOOST");
            }

            // 2. Condición Principal: Empuje sostenido en tiempo + Salida física de rampa
            if (timestamp_millis_inicio_pico_g != 0) {
                const uint32_t duracion_pico_ms = ahora_ms - timestamp_millis_inicio_pico_g;

                Serial.printf("datos_sensores->altitud_filtrada_m_bmp=%f\n", datos_sensores->altitud_asl_filtrada_m_bmp);
                Serial.printf("SYSTEM.ctx_fisico.altitud_m_pad=%f\n", SYSTEM.ctx_fisico.altitud_m_pad);
                Serial.printf("SYSTEM.ctx_fisico.altitud_m_relativa_al_pad=%f\n", SYSTEM.ctx_fisico.altitud_m_relativa_al_pad);
                if ((duracion_pico_ms >= TIEMPO_MS_MIN_BOOST) && (SYSTEM.ctx_fisico.altitud_m_relativa_al_pad > ALTITUD_MIN_SALIDA_RAMPA_M)) {
                    EnlaceGSE::enviarMensaje("BOOST Confirmado");
                    ESP_LOGI(TAG_BASE, "BOOST Confirmado");
                    timestamp_millis_inicio_pico_g = 0;
                    timestamp_ultima_acel_valida = 0;
                    return true; // Despegue nominal confirmado (IMU + Barómetro)
                }
            }

            // 3. Condición de Redundancia / Fallback Robusta (Fallo de acelerómetro)
            if (SYSTEM.ctx_fisico.altitud_m_relativa_al_pad > ALTITUD_FALLBACK_M) {
                EnlaceGSE::enviarMensaje("BOOST Confirmado por fallback (diferencia de altitud");
                ESP_LOGI(TAG_BASE, "BOOST Confirmado por fallback (diferencia de altitud");
                timestamp_millis_inicio_pico_g = 0;
                timestamp_ultima_acel_valida = 0;
                return true; // Despegue confirmado por cinemática pura
            }

            return false;
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * @brief Timer para Esperar a que la IMU se caliente --> pasados los 5 mins recalibramos timer.
     */
    void f_st_init(data_all_t* datos_sensores, uint32_t ms_en_estado) {

        //TODO: LÓGICA DE LECTURA DE FLASH = X
        // Note: Delegado en main, máxima prioridad

        // Esperar 5 minutos para que la mpu entre en calor, luego calibrarla.
        if (Timers::xTimerRecalibrarMPU == NULL) {
            Timers::xTimerRecalibrarMPU =
                xTimerCreate(
                    "Recalibrar",
                    pdMS_TO_TICKS(TIEMPO_MILLIS_ESPERA_WARMUP_MPU),
                    pdFALSE, // one shot timer
                    nullptr,
                    Timers::calibrar_mpu_callback
                );
        }

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

        // asume que cuando el cohete es encendido, ya se encuentra sobre la rampa.
        auto configurar_altura_rampa = [](void* pvParameters) {
            SYSTEM.ctx_fisico.altitud_m_pad = Sensors::getBMP280().get_altitude_media_iterations(); // esta cosa es bloqueante!
            ESP_LOGI("configurar_altura_rampa", "Altitud ASL de rampa: %f m\n", SYSTEM.ctx_fisico.altitud_m_pad);
            CONFIG_RESTAURACION.altitud_del_pad = SYSTEM.ctx_fisico.altitud_m_pad;
            vTaskDelete(NULL);
        };
        xTaskCreatePinnedToCore(configurar_altura_rampa, "calcular_altura_rampa", 2*1024, NULL, ConfigInit::TASK_PRIORITY_COMMON, NULL, 1);

        // Se debería llamar a la función _lora.c_connect_to_GSE() por medio de GSE o EnlaceGSE,
        //_lora.c_connect_to_GSE(); EMITE UN PING

        // CONFIG_RESTAURACION.mision_activa = true; // TODO: Esta bien afirmar que mision activa durante salida del ST_INIT ?
        SYSTEM.estado = ST_ESPERA_CONEXION_GSE;
        // transicionar_hacia(ST_ESPERA_CONEXION_GSE);
    }

    void f_st_espera_conexion_gse(data_all_t* datos_sensores, uint32_t ms_en_estado) {

        if (ms_en_estado == 0)
            ESP_LOGI(TAG_BASE, "Esperando conexión con la GSE...");

        //  Condición de éxito: ¿Se conectó por LoRa?
        if (GSE::get_estado_conexion() == ROCKET_CONNECTED) {
            ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Nos conectamos exitósamente.");
            EnlaceGSE::enviarMensaje("Conexion establecida.");
            SYSTEM.estado = ST_ESPERA_GPS_PRECISO;
            return;
        }

        // Condición de Timeout: Evaluado directamente contra el argumento
        if (ms_en_estado >= CONEXION_GSE_TIMEOUT_MILLIS) {
            ESP_LOGE(TAG_BASE, "Timeout de conexión con GSE. Continuando vuelo...");
            SYSTEM.estado = ST_ESPERA_GPS_PRECISO;
        }
    }


    void f_st_espera_gps_preciso(data_all_t* datos_sensores, uint32_t ms_en_estado) {

        if (ms_en_estado == 0)
            ESP_LOGI(TAG_BASE, "Esperando condiciones estables para el módulo GPS...");

        // Timeout
        if (ms_en_estado >= GPS_TIMEOUT_MILLIS) {  // TODO: Deberiamos poner Timeout largo teniendo en cuenta que ya se puede overridear desde GSE
            ESP_LOGE(TAG_BASE, "Timeout de GPS. Avanzando a espera de ignición.");
            EnlaceGSE::enviarError("[GPS] Timeout");
            // SYSTEM.flags.gps_preciso = false; // no se usará esta condicion luego
            SYSTEM.estado = ST_ESPERA_IGNICION;
            return;
        }

        // SYSTEM.flags.gps_preciso = Eventos::gps_es_preciso(datos_sensores); // este flag es luego utilizada en Evento::en_condiciones_para_volar --> UPDATE: GPS no es indispensable.

        // Condición de éxito o bypass de GSE
        if (SYSTEM.gse.gps_override_skip || Eventos::gps_es_preciso(datos_sensores)) {
            const auto msg = SYSTEM.gse.gps_override_skip?
                    "[GPS] SKIP búsqueda de precisión de gps":
                    "[GPS] Precisión asegurada.";
            ESP_LOGI(TAG_BASE, " -> %s", msg);
            EnlaceGSE::enviarMensaje(msg);
            SYSTEM.estado = ST_ESPERA_IGNICION;
            return;
        }

        // LOG
        if (ms_en_estado % 1000 <= 10){
            ESP_LOGI(TAG_BASE, " -> [GPS] Esperando satélites. Visibles: %d", datos_sensores->gps_nro_satelites);
        }
    }

    void f_st_espera_ignicion(data_all_t* datos_sensores, uint32_t ms_en_estado) {
        static uint32_t timer0_millis = 0;
        // static uint32_t timer1_millis = 0;
        // Aquí el cohete está pasivo en la rampa. Ignición es externa.
        // Lógica del filtro anti-zarandeo:
        // 1. Transformar aceleración a vector inercial.
        // 2. if (aceleracion_vertical >= 2.0g) {
        //      iniciar temporizador (timestamp_inicio_pico_g)
        //    }
        // 3. if (tiempo_con_2g >= 150ms AND delta_altura > 4.0m) {
        //      Transición a ST_PROPULSION.
        //    }

        const bool lora_conectado_con_gse = (GSE::get_estado_conexion() == ROCKET_CONNECTED);
        if (lora_conectado_con_gse) {
            // Si hay enlace GSE
            if (!SYSTEM.flags.flash_log_borrado) {
                // Dispara la orden SOLO SI los logs del flash no fueron borrados previamente
                // TODO: MOVER RESPONSABILIDAD DE FLASH_BORRADO_O_NO A LA FLASH.
                // Notifica a la tarea de Flash para que despierte ya mismo
                if (SYSTEM.procesos.xTaskFlashHandle != NULL) {
                    xTaskNotifyGive(SYSTEM.procesos.xTaskFlashHandle);
                    ESP_LOGW(TAG_BASE, "Autoborrado de Flash solicitado por MdE en rampa...");
                }
            }
        }

        bool en_codiciones_para_volar = false;

        const bool apuntando_al_cielo = Eventos::entorno(datos_sensores->angulo_respecto_z_deg, 0.0f, 5.0f);
        if (!apuntando_al_cielo) {
            if (ms_en_estado % 1000 <= 10) {
                ESP_LOGI(TAG_BASE, " -> [f_st_espera_ignicion] PELIGRO. COHETE NO APUNTANDO AL CIELO. AnguloDeInclinacion=%f", datos_sensores->angulo_respecto_z_deg);
                EnlaceGSE::enviarMensaje("[f_st_espera_ignicion] PELIGRO. COHETE NO APUNTANDO AL CIELO.");
                // PODEMOS ACTIVAR EL BUZZER PARA QUE MOLESTE MUCHO.
                Actuators::getBuzzer().playError();
            }
        }

        // Por requerimientos --> no dependemos de Lora o GPS para volar.
        en_codiciones_para_volar = (apuntando_al_cielo && Timers::flag_recalibrarMPU_disparado);

        if(en_codiciones_para_volar) {
            // if led no encendido: encenderlo para señalizar que ya podemos volar.
            if (millis() - timer0_millis >= 3000) { // cada 3 segundos hacer un beep
                timer0_millis = millis();
                // buzzer.beep(100);
                ESP_LOGI(TAG_BASE, " -> [ESPERA IGNICION] Cohete en condiciones para volar!");
            }
        }

        if (Eventos::hay_boost_garantizado(datos_sensores)) {
            if (!en_codiciones_para_volar) {
                // transicion_error(ERR_DESPEGUE_PROHIBIDO, datos_sensores);
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_DESPEGUE_PROHIBIDO");
                // L: Qué hacemos si realmente detectamos un despegue, pero el cohete no estaba en condiciones de volar? Pienso que: ya que esta en vuelo, mucho no podemos hacer al respecto, hay que continuar con lo que se tiene. Ver qué hacemos a partir de ahí.
                // J: TOTALMENTE. Que sea lo que Dios quiera

                EnlaceGSE::enviarError("Despegue no autorizado");
            }

            SYSTEM.estado = ST_BOOST;
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
        
        if (datos_sensores->angulo_respecto_z_deg > 30.0f ) { // por requerimientos --> 30 grados de inclinacion
            if (ticks_inclinacion_peligrosa < 255) ticks_inclinacion_peligrosa++;
        } else {
            ticks_inclinacion_peligrosa = 0;
        }

        if (ticks_inclinacion_peligrosa >= 3) {
            EnlaceGSE::enviarError("[ABORT] INCLINACION CRITICA. FORZANDO ABORTO.");
            
            // Disparar pirotécnicos para desestabilizar el cohete
            Actuators::getPyroDrogue().armar();
            Actuators::getPyroDrogue().disparar();
            
            SYSTEM.estado = ST_CAIDA_CATASTROFICA; // O un estado específico de ABORTO
            return; 
        }

        // ---------------------------------------------------------
        // 2. DETECCIÓN DE BURN-OUT (Transición a Balística)
        // ---------------------------------------------------------
        // Ignoramos el primer tramo (ej. 5000ms) donde la curva de empuje 
        // podría tener fluctuaciones iniciales propias del motor sólido.
        if (ms_en_estado > 1000) {
            
            // Si la aceleración Y (sin gravedad) cae por debajo de 0,
            // significa que el empuje es menor que el Drag. El motor se apagó.
            if (datos_sensores->aceleracion_vertical_m_s2_mpu <= 0.0f) {
                
                // Verificamos que sigamos subiendo a buena velocidad como doble chequeo
                if (datos_sensores->velocidad_vertical_filtrada_m_s > 5.0f) {
                    EnlaceGSE::enviarMensaje("[BOOST] MOTOR APAGADO. MODO BALÍSTICO.");
                    ESP_LOGI(TAG_BASE, "[BOOST] MOTOR APAGADO. MODO BALÍSTICO.");

                    SYSTEM.estado = ST_FASE_BALISTICA;
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
    if ((datos_sensores->velocidad_vertical_filtrada_m_s > -0.2f) &&
        (datos_sensores->altitud_asl_filtrada_m_bmp > SYSTEM.ctx_fisico.altura_m_max_historica))
    {
        SYSTEM.ctx_fisico.altura_m_max_historica = datos_sensores->altitud_asl_filtrada_m_bmp;
    }
    
    // ---------------------------------------------------------
    // 2. GATES DE DETECCIÓN DE APOGEO
    // ---------------------------------------------------------  
  	const bool vel_apogeo = (datos_sensores->velocidad_vertical_filtrada_m_s <= -0.3f);
    const bool caida_confirmada = (SYSTEM.ctx_fisico.altura_m_max_historica - datos_sensores->altitud_asl_filtrada_m_bmp) >= DIFF_ALTURA_M_APOGEO_CAIDA;
    const bool g_gate_valido = (datos_sensores->aceleracion_vertical_m_s2_mpu > -5.0f && datos_sensores->aceleracion_vertical_m_s2_mpu < 2.0f);

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
                SYSTEM.estado = ST_CAIDA_CATASTROFICA;
                return;
            }
        } 
        // B) El disparo ya se ordenó. Mantenemos el MOSFET 100ms activo (Asíncrono)
        else {
            if ((micros() - SYSTEM.timestamp_micros_apertura_drogue) >= 100000UL) {
                
                if (SYSTEM.flags.drogue_disparado) {
                    Actuators::getPyroDrogue().desarmar();
                    ESP_LOGI(TAG_BASE, "[APOGEO] Drogue ignisecus OK. Transición a ST_DROGUE_DESPLEGADO");
                    SYSTEM.estado = ST_DROGUE_DESPLEGADO;
                } else {
                    Actuators::getPyroPpal().desarmar();
                    ESP_LOGI(TAG_BASE, "[APOGEO] Principal ignisecus OK. Transición a ST_PCAIDAS_PPAL_DESPLEGADO");
                    SYSTEM.estado = ST_PCAIDAS_PPAL_DESPLEGADO;
                }
                return;
            }
        }
    } 
    // 4. CONTROL DE AIRBRAKES DURANTE ASCENSO SIN BOOST
    // else {
    //     float angulo_airbrake = 0.0f;
    //     if (datos_sensores->velocidad_vertical_filtrada_m_s >= 15.0f) {
    //         // angulo_airbrake = MPC_Controller::calcular_accion(...);
    //     }
    //     Actuators::getServo().sendAngulo(angulo_airbrake);
    // }
}


void f_st_drogue_desplegado(data_all_t* datos_sensores, uint32_t ms_en_estado) {
    static uint8_t ticks_despliegue_ppal = 0;
    static bool disparo_ppal_iniciado = false;
    static uint32_t timestamp_inicio_pulso_ppal = 0;

    if (ms_en_estado == 0) {
        ticks_despliegue_ppal = 0;
        disparo_ppal_iniciado = false;
    }

    // ---------------------------------------------------------
    // 1. VERIFICACIÓN DE SALUD DEL DROGUE (A los 3 segundos)
    // ---------------------------------------------------------
    // Si caemos demasiado rápido (ej. vel < -35 m/s), el drogue falló o se desprendió.
    if (ms_en_estado >= 3000 && !disparo_ppal_iniciado) {
        if (datos_sensores->velocidad_vertical_filtrada_m_s < -35.0f) {
            ESP_LOGE(TAG_BASE, "[ALERTA] Caída excesivamente rápida (%.1f m/s). ¡Drogue falló!", datos_sensores->velocidad_vertical_filtrada_m_s);
            // Adelantamos la apertura del principal por emergencia
            ticks_despliegue_ppal = 10; 
        }
    }

    // ---------------------------------------------------------
    // 2. CONDICIÓN NOMINAL: ALTITUD DE DESPLIEGUE DEL PRINCIPAL
    // ---------------------------------------------------------
    if (SYSTEM.ctx_fisico.altitud_m_relativa_al_pad <= ALTITUD_DESPLIEGUE_PCAIDAS_PPAL && datos_sensores->velocidad_vertical_filtrada_m_s < 0.0f) {
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
                timestamp_inicio_pulso_ppal = micros();
            } else {
                ESP_LOGE(TAG_BASE, "[CRÍTICO] Sin continuidad en Pyro Principal. Transición a Caída Catastrófica.");
                SYSTEM.estado = ST_CAIDA_CATASTROFICA;
                return;
            }
        } 
        else {
            // Mantenemos el pulso de 100 ms antes de cortar corriente y transicionar
            if ((micros() - timestamp_inicio_pulso_ppal) >= 100000UL) {
                Actuators::getPyroPpal().desarmar();
                ESP_LOGI(TAG_BASE, "[DESCENSO] Principal desplegado con éxito.");
                SYSTEM.estado = ST_PCAIDAS_PPAL_DESPLEGADO;
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
    const bool velocidad_nula = (datos_sensores->velocidad_vertical_filtrada_m_s > -0.4f && datos_sensores->velocidad_vertical_filtrada_m_s < 0.4f);
    
    // Condición B: Aceleración estática (sin sacudidas del paracaídas ni viento fuerte)
    const bool aceleracion_estatica = (datos_sensores->aceleracion_vertical_m_s2_mpu > -1.5f && datos_sensores->aceleracion_vertical_m_s2_mpu < 1.5f);

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
        SYSTEM.estado = ST_ATERRIZADO;
    }
}

    void f_st_caida_catastrofica(data_all_t* datos_sensores, uint32_t ms_en_estado) {

        // Solo ejecutamos las acciones críticas al entrar al estado
        if (ms_en_estado == 0) {
            ESP_LOGE(TAG_BASE, "==================================================");
            ESP_LOGE(TAG_BASE, " -> [FATAL] CAÍDA CATASTRÓFICA DETECTADA.");
            ESP_LOGE(TAG_BASE, " -> INICIANDO PROTOCOLO DE EMERGENCIA POST-MORTEM.");
            ESP_LOGE(TAG_BASE, "==================================================");

            // 1. Activar bandera global de pánico cargar datos
            SYSTEM.flags.emergencia_fatal = true;
            // SYSTEM.datos_actuales.gps_latitud   = datos_sensores->gps_latitud; // ya lo actualiza de prepo mde_cohete_actualizar(...)
            // SYSTEM.datos_actuales.gps_longitud  = datos_sensores->gps_longitud;

            // 2. Medida desesperada: Forzar disparo del principal si tiene continuidad
            // Si ya se disparó antes o está roto, esto fallará de forma segura.
            // Si por algún bug de software no se había abierto, esto podría salvar el fuselaje.
            if (Actuators::getPyroPpal().tieneContinuidad()) {
                Actuators::getPyroPpal().armar();
                Actuators::getPyroPpal().disparar();
            }

            // 3. Ordenar el volcado de RAM a Flash Interna (LittleFS/SPIFFS)
            // SYSTEM.accion.volcar_ram_a_flash = true;
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

         // Solo ejecutamos las acciones críticas al entrar al estado
        if (ms_en_estado == 0) {
            ESP_LOGE(TAG_BASE, "==================================================");
            ESP_LOGE(TAG_BASE, " -> [EXITO] ATERRIZAJE CORRECTO.");
            ESP_LOGE(TAG_BASE, "==================================================");

            // 1. Activar bandera global de pánico  cargar datos
            SYSTEM.flags.emergencia_fatal = true;
            // SYSTEM.datos_actuales.gps_latitud   = datos_sensores->gps_latitud;  // ya lo actualiza de prepo mde_cohete_actualizar(...)
            // SYSTEM.datos_actuales.gps_longitud  = datos_sensores->gps_longitud;


            // 2. Ordenar el volcado de RAM a Flash Interna (LittleFS/SPIFFS)
            SYSTEM.flags.volcar_ram_a_flash = true;
            if (SYSTEM.procesos.xTaskFlashHandle != NULL) {
                xTaskNotifyGive(SYSTEM.procesos.xTaskFlashHandle);
            }

            // 3. Despertar a la tarea LoRa INMEDIATAMENTE para iniciar el SOS
            if (SYSTEM.procesos.xTaskLoraHandle != NULL) {
                xTaskNotifyGive(Cohete::SYSTEM.procesos.xTaskLoraHandle);
            }
        }

        // 2. Detener logs de alta frecuencia para salvar batería.


        if(ms_en_estado % 500 <= 5){
            // Suponiendo que los sensores funcionan... MEJOR NO xd
            // SYSTEM.datos_actuales.gps_longitud  = datos_sensores->gps_longitud;  // ya lo actualiza de prepo mde_cohete_actualizar(...)
            // SYSTEM.datos_actuales.gps_latitud   = datos_sensores->gps_latitud;
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
