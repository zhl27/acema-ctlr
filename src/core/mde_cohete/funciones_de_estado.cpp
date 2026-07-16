#include "funciones_de_estado.h"

#include "mde_cohete.h"
#include "SerialPrint.h"
#include "esp_log.h"
#include "services/Actuators.h"

#include "services/GSE.h"
#include "services/Sensors.h"
#include  "include.h"


// TODO: Integrar con la variable global COHETE para transiciones de estado

constexpr float A_GRAV = 9.81;
constexpr float UMBRAL_ACEL_BOOST = 2 * A_GRAV;     // 2G según requerimientos
constexpr int TIEMPO_MIN_BOOST_MS = 150;            // 0.15 segundos en milisegundos
constexpr float DELTA_ALTURA_BOOST_M = 4.0f;        // Delta de seguridad contra falsos positivos en rampa // TODO: Revisar este 4
constexpr float PESO_KG_COMBUSTIBLE = 5;            // TODO: COMPLETAR CON EL DATO REAL
constexpr uint32_t CONEXION_GSE_TIMEOUT_MILLIS = 1000*5;
constexpr float ALTURA_M_MAX = 1000; // TODO: chequear ALTURA_M_MAX. Igual nos importa realmente este dato?
constexpr uint32_t GPS_TIMEOUT_MILLIS = 1000*5;
constexpr uint32_t TIEMPO_MILLIS_ESPERA_WARMUP_MPU = 1000*60*5;



namespace Cohete {

    namespace Timers
    {
        static TimerHandle_t xTimerRecalibrarMPU;
        static bool flag_recalibrarMPU_disparado = false;

        // typedef void (* TimerCallbackFunction_t)( TimerHandle_t xTimer );
        void calibrar_mpu_callback(TimerHandle_t xTimer) {
            Sensors::getMPU6050().calibrar();
            flag_recalibrarMPU_disparado = true;
            ESP_LOGI(TAG_BASE, "Temporizador xTimerRecalibrarMPU disparado!");
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    namespace Eventos {
        // val0 --> valor de estudio
        // val1 --> valor objetivo
        // delta --> margen
        inline bool aproxima(const int val0, const int val1, const int delta) {
            return (val0 <= val1 + delta && val0 >= val1 - delta);
        }
        // lo mismo que decir: es val0 menor por 5 unidades a val1?
        // inline bool menor_delta_que(const int val0, const int val1, const float delta) {
        //     return (val0 <= val1 + delta);
        // }

        inline bool gps_es_preciso(const data_all_t* datos_sensores) {
            return datos_sensores->gps_nro_satelites >= 5       // Mínimo 4 para 3D, 5 o 6 es más seguro
                && datos_sensores->gps_fix_type == 3            // Equivalente a 3D Fix en u-blox (fixType == 3)
                && datos_sensores->gps_gnss_fix_ok == true      // ¡CRÍTICO! El flag del módulo que confirma que el arreglo es válido
                && datos_sensores->gps_pdop <= 2.0;              // Dilución de precisión (pDOP * 0.01f) menor o igual a 2.0
            // && datos_sensores->gps_hacc <= 2500; // ¡EXTRA! Precisión horizontal (hAcc) menor a 2.5 metros (2500 mm)
        }

        bool en_codiciones_para_volar(data_all_t* datos_sensores) {
            return Timers::flag_recalibrarMPU_disparado; // este flag nos permite asumir que los datos MPU son aceptables
            // TODO: COMPLETAR CONDICIONES PARA VUELO.
            // queremos sí o sí el GPS para el vuelo?
        }

        bool hay_boost(data_all_t *datos_sensores) { // TODO: Completar lógica de Boost
            // aceleracion >= 2 g por 0,15 segs
            if (datos_sensores->aceleracion_z_m_s2 >= UMBRAL_ACEL_BOOST) { // detectamos un supuesto boost, chequeamos...
                if (SYSTEM.timestamp_millis_inicio_pico_g == 0) { // que sea 0 significa que nunca antes habiamos detectado inicio de boost --> imposible que tengamos boost en micros()==0
                    SYSTEM.timestamp_millis_inicio_pico_g = millis(); // detectamos un pico por primera vez y guardamos timestamp
                }
                else if (SYSTEM.timestamp_millis_inicio_pico_g <= millis() && millis() - SYSTEM.timestamp_millis_inicio_pico_g >= TIEMPO_MIN_BOOST_MS) {
                    if (datos_sensores->altitud_filtrada_m > SYSTEM.ctx_fisico.altitud_cero_pad_m + DELTA_ALTURA_BOOST_M) {
                        return true;
                    }
                }
            } else {
                // CRÍTICO: Si la aceleración cae por debajo de 2 g antes de confirmar el vuelo,
                // se trató de un ruido, un golpe o un movimiento brusco manual.
                // Reiniciamos el cronómetro a 0 para estar listos para el despegue real.
                SYSTEM.timestamp_millis_inicio_pico_g = 0;
            }

            return false;
        }

    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void f_st_init(data_all_t* datos_sensores) {
        ESP_LOGI(TAG_BASE, "INIT");

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
        }

        transicionar_hacia(ST_ESPERA_CONEXION_GSE);
    }

    void f_st_espera_conexion_gse(data_all_t* datos_sensores) {
        static uint64_t timestamp_millis_inicio_timeout;

        // Lógica:
        // 1. Intentar recibir configuración por LoRa (GSE).
        // 2. Si llegan los datos, configurar y pasar a ST_ESPERA_DESPEGUE.
        // 3. Si (millis() - t_inicio_busqueda > TIMEOUT_LORA), entonces:
        //    COHETE.vuelo_en_silencio_radio = true;
        //    Transición forzada a ST_ESPERA_DESPEGUE (el despegue es prioridad).
        if (entrando_a_estado()) {
            if (GSE::estado_conexion_gse() == ROCKET_DISCONNECTED || GSE::estado_conexion_gse() == ROCKET_INIT) {
                timestamp_millis_inicio_timeout = millis();
                ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Esperando conexión con GSE...");
            }

        }
        else if (GSE::estado_conexion_gse() == ROCKET_WAITING_PONG) {
            // TODO: Qué hacemos mientras cohete espera conexión con GSE ?
            if ((millis() - timestamp_millis_inicio_timeout) >= CONEXION_GSE_TIMEOUT_MILLIS) { // TODO: Revisar si el timeout del gse es conveniente
                transicion_error(ERR_TIMEOUT_CONEXION_GSE, datos_sensores);
            }
        }
        else if (GSE::estado_conexion_gse() == ROCKET_CONNECTED) {
            ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Nos conectamos a la GSE.");
            transicionar_hacia(ST_ESPERA_GPS_PRECISO);
        }
    }

    void f_st_espera_gps_preciso(data_all_t* datos_sensores) {
        static uint64_t timestamp_millis_inicio_timeout;

        if (entrando_a_estado()) {
            timestamp_millis_inicio_timeout = millis();
        }
        else if ((millis() - timestamp_millis_inicio_timeout) >= GPS_TIMEOUT_MILLIS) {
            transicion_error(ERR_GPS_TIMEOUT, datos_sensores);
        }
        if (datos_sensores->gps_nro_satelites < 4) {
            if (millis() % 200 == 0)
                ESP_LOGI(TAG_BASE, " -> [GPS] Esperando satélites. Visibles: %d", datos_sensores->gps_nro_satelites);
        }
        else if (Eventos::gps_es_preciso(datos_sensores)) {
            ESP_LOGI(TAG_BASE, " -> [GPS] Precisión de GPS asegurado!");
            transicionar_hacia(ST_ESPERA_IGNICION);
        }
    }



    void f_st_espera_ignicion(data_all_t* datos_sensores) {
        static unsigned long last_millis = 0;
        // Aquí el cohete está pasivo en la rampa. Ignición es externa.
        // Lógica del filtro anti-zarandeo:
        // 1. Transformar aceleración a vector inercial.
        // 2. if (aceleracion_vertical >= 2.0g) {
        //      iniciar temporizador (timestamp_inicio_pico_g)
        //    }
        // 3. if (tiempo_con_2g >= 150ms AND delta_altura > 4.0m) {
        //      Transición a ST_PROPULSION.
        //    }
        if (entrando_a_estado()) {
            ESP_LOGI(TAG_BASE, "[ESPERA IGNICION] Esperando condiciones necesarias para el vuelo...");
        }

        if(Eventos::en_codiciones_para_volar(datos_sensores)) {
            // if led no encendido: encenderlo para señalizar que ya podemos volar.
            if (millis() - last_millis >= 3000) {
                last_millis = millis();
                // buzzer.beep(100);
                ESP_LOGI(TAG_BASE, " -> [ESPERA IGNICION] Cohete en condiciones para volar!");
            }

            if (Eventos::hay_boost(datos_sensores)) {
                // SYSTEM.timestamp_millis_inicio_pico_g = micros();
                transicionar_hacia(ST_BOOST);
            }
        }
        else if (Eventos::hay_boost(datos_sensores)) { // detectamos boost a pesar de no estar en condiciones para volar, KEEEEE!!!!!!
            transicion_error(ERR_DESPEGUE_PROHIBIDO, datos_sensores);
        }
    }

    void f_st_boost(data_all_t* datos_sensores) {
        // como son static se crean una sola vez
        static float altura_entrada_st_boost;
        static float aceleracion_z_entrada_st_boost;
        static float velocidad_z_entrada_st_boost;

        if (entrando_a_estado()) {
            altura_entrada_st_boost = datos_sensores->altitud_filtrada_m;
            aceleracion_z_entrada_st_boost = datos_sensores->aceleracion_z_m_s2;
            velocidad_z_entrada_st_boost = datos_sensores->vel_z_filtrada_m_s;
        }
        else if ((datos_sensores->altitud_filtrada_m - altura_entrada_st_boost) >= DELTA_ALTURA_BOOST_M) { // TODO: DEFINIR BIEN LA CONDICION de DIFF ALTURAS, PARA RESPALDAR QUE ENTRAMOS A BOOST REALMENTE.
            // si la diferencia no es considerable como para respaldar que estamos definitivamente en modo BOOST...
            transicion_error(ERR_DESPEGUE_FALSO_ZARANDEO, datos_sensores);
            return;
        }

        // Cuando la aceleración vertical decaiga bruscamente (Burn-out / Fin de combustión)
        // if (datos_sensores-> altura_m < altura_entrada_st_boost) { --> esto no determina comienzo de fase balistica, es parte de, sí, pero no determina. --> la fase balistica comienza con una velocidad hacia arriba que va desacelerandose por la gravedad --> al comienzo, la altura sigue incrementandose, aunque a tasas cada vez menores, hasta que empezamos a tener velocidad hacia abajo.
        if ((aceleracion_z_entrada_st_boost - datos_sensores->aceleracion_z_m_s2) > -A_GRAV) { // Detectamos que hubo una DESaceleracion.
            // comparamos velocidad en el tiempo
            // TODO: Debemos comprobar que realmente hubo una DESaceleracion
            // usar velocidad para double-check --> no nos confirma nada, ya que la velocidad la calculamos a partir de la aceleracion.
            // TODO: Deberiamos comprobarlo con otros datos, quizas el GPS sea nuestro mejor aliado en este problema.
            // TODO: utilizar la altura (calculada a partir de la presion de la bmp) para verificar que hay un decremento en la tasa de cambio de la altura, es decir, que la altura sube cada vez más lento, hasta que su tasa de cambio se vuelva cero (implica que alcanzó apogeo)
            if ((velocidad_z_entrada_st_boost - datos_sensores->vel_z_filtrada_m_s) > 0) {
                SYSTEM.ctx_fisico.masa_cohete_kg -= PESO_KG_COMBUSTIBLE; // TODO: asumimos que el combustible se consumio completamente ?
                transicionar_hacia(ST_FASE_BALISTICA);
            }
        }
    }

    void f_st_fase_balistica(data_all_t* datos_sensores) {
        // Lógica:
        // 1. Activar servomotores de frenado aerodinámico si están integrados.
        // 2. Monitorear constantemente las condiciones de Apogeo:
        //    if (velocidad_vertical <= 0 && aceleracion_vertical < 0 && altura_actual == altura_maxima) {
        //      Transición a ST_APOGEO.
        //    }


        if (SYSTEM.ctx_fisico.altura_m_max_historica < datos_sensores->altitud_filtrada_m) {
            // encontramos nueva altura historica
            SYSTEM.ctx_fisico.altura_m_max_historica = datos_sensores->altitud_filtrada_m;  // esto va buscando constantemente el apogeo
        }

        // DETECCIÓN DE APOGEO:
        // 1. La velocidad vertical se vuelve <= 0 m/s (con un pequeño margen anti-ruido, ej -0.5 m/s)
        // 2. La altitud actual cayó al menos 1.5 metros desde el máximo histórico
        // 3. En vuelo libre, el acelerómetro lee cerca de 0 m/s² (entre -5 m/s² y +2 m/s²)
        const bool vel_negativa = (datos_sensores->vel_z_filtrada_m_s <= -0.5f);
        const bool caida_confirmada = (SYSTEM.ctx_fisico.altura_m_max_historica - datos_sensores->altitud_filtrada_m) >= 1.5f;
        const bool en_caida_libre = (datos_sensores->aceleracion_z_m_s2 > -6.0f && datos_sensores->aceleracion_z_m_s2 < 3.0f);
        if ((vel_negativa || caida_confirmada) && en_caida_libre) // TODO: TENER EN CUENTA VIENTO ETC ETC --> QUIZAS NO SEA SOLO GRAVEDAD
        {
            if (!Actuators::getPyroDrogue().tieneContinuidad()) {
                // aproxima(datos_sensores->altura_m, ALTURA_M_MAX, 5) // TODO: Ver que cosas interesantes se puede hacer con esto.
                Actuators::getPyroDrogue().armar();
                Actuators::getPyroDrogue().disparar();
            }
            else { // se encendió efectivamente el pirotécnico, podemos cambiar de estado.
                SYSTEM.timestamp_micros_apertura_drogue = micros();
                transicionar_hacia(ST_DESPLIEGUE_DROGUE);
            }
        }
        else { // Solo usamos el airbrake si todavia no llegamos a apogeo (que es cuando se desplega el drogue)
            constexpr float angulo = 0.0f; // TODO: Acá iría la función de airbrake_mpc
            Actuators::getServo().setAngulo(angulo);
        }
    }

    void f_st_despliegue_drogue(data_all_t* datos_sensores) { // TODO: ST_APOGEO quizás no sea necesario, es más bien un evento dentro de ST_FASE_BALISTICA
        // Lógica:
        // 1. Enviar señal de STOP a la cámara por pin/UART.
        // 2. Ignición pirotécnica del Drogue.
        // 4. Transición inmediata a ST_DESCENSO_EVALUACION.

        // Chequeamos que el paracaidas drogue realmente se desplegó
        // 1. Medimos continuidad del pyro
        // 2. Medimos que la velocidad sea constante (con cierto ruido que debemos ignorar) gracias al drogue haciendo friccion con el aire. --> equivalente seria medir que aceleracion aproxima a 0.
        if (entrando_a_estado()) {
            if (Eventos::aproxima(datos_sensores->aceleracion_z_m_s2, 0, 5)) {
                ESP_LOGI(TAG_BASE, " -> [%s] Drogue desplegado correctamente.", estado_cohete_string[SYSTEM.estado]);
                transicionar_hacia(ST_DESCENSO_EVALUACION);
            }
        }
        // chequear que realmente llegamos a apogeo
        // comprobar que altura paso por un punto más alto y descendio inmediatamente
        else if (datos_sensores->vel_z_filtrada_m_s <= 0) { // está cayendo
            if (datos_sensores->altitud_filtrada_m < SYSTEM.ctx_fisico.altura_m_max_historica) {
                transicionar_hacia(ST_DESCENSO_EVALUACION);
            }
        }
    }

    void f_st_evaluar_supervivencia_drogue(data_all_t* datos_sensores) {
        // Este estado DEBE durar exactamente 3 segundos.
        // Lógica:
        // if (millis() - COHETE.timestamp_apertura_drogue >= 3000) {
        //      if (velocidad_vertical <= -35 m/s) {
        //          // Falló el drogue
        //          Transición a ST_DESCENSO_EMERGENCIA;
        //      } else if (aceleracion_vertical ~= -1g) {
        //          // Caída libre balística. Muerte inminente.
        //          Transición a ST_CAIDA_CATASTROFICA;
        //      } else {
        //          // Todo en orden. El drogue frenó el cohete.
        //          Transición a ST_DESCENSO_NOMINAL;
        //      }
        // }
    }

    void f_st_descenso_controlado_drogue(data_all_t* datos_sensores) {
        // Bajando con Drogue.
        // Lógica:
        // if (altura_actual_filtrada <= 250.0m) { // ¡Cuidado de chequear contra cota_suelo_rampa!
        //      Ignición pirotécnica Paracaídas Principal.
        //      // ESP_LOGI(TAG_STATE_MACHINE, " -> [NOMINAL] Paracaídas Principal desplegado.");
        //      Transición a ST_ATERRIZAJE (o estado intermedio de espera).
        // }
    }

    void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores) {
        ESP_LOGI(TAG_BASE, " -> [EMERGENCIA] Drogue fallido. Disparando Principal de inmediato!");
        // Lógica:
        // 1. Disparo inmediato del paracaídas principal.
        // 2. Transición a ST_ATERRIZAJE (esperando el suelo).
    }

    void f_st_ejecutar_panico_flash_dump(data_all_t* datos_sensores) {
        ESP_LOGI(TAG_BASE, " -> [FATAL] Caída libre detectada. Volcando RAM a Flash!");
        // Lógica:
        // 1. Las tarjetas SD mecánicas pueden corromperse en impactos duros.
        // 2. Escribir el buffer circular de últimos 5 segundos en la Flash de la ESP32
        //    (NVS o SPIFFS/LittleFS) para análisis post-mortem.
        // 3. Este estado no tiene salida, es terminal antes del impacto.
    }

    void f_st_aterrizaje(data_all_t* datos_sensores) {
        // Lógica:
        // 1. Detectamos reposo en el suelo (posicion_world cerca de 0 relativa, acel == 0).
        // 2. Detener logs de alta frecuencia para salvar batería.
        // 3. Cerrar archivos en la SD (flush y close).
        // 4. Emitir un "beep" continuo y transmitir coordenadas Lat/Lon por LoRa cada X segundos.
    }

    void f_st_error(data_all_t* datos_sensores) {

        // MANEJO GENERICO DE FALLAS

        ESP_LOGE(TAG_BASE, "[ST_ERROR]");
        ESP_LOGE(TAG_BASE, "ERROR DESCONOCIDO SIN MANEJAR.");
        ESP_LOGE(TAG_BASE, "SYSTEM.error=%d", SYSTEM.error);

    }

}
