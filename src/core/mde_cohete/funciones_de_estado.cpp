#include "funciones_de_estado.h"

#include "mde_cohete.h"
#include "SerialPrint.h"
#include "services/Actuators.h"
#include "services/GSE.h"
#include "services/Sensors.h"



// TODO: Integrar con la variable global COHETE para transiciones de estado

constexpr float A_GRAV = 9.81;
constexpr float PESO_KG_COMBUSTIBLE = 5; // TODO: COMPLETAR CON EL DATO REAL
constexpr uint64_t CONEXION_GSE_TIMEOUT_MILLIS = 1000*5;
constexpr float ALTURA_M_MAX = 1000;
constexpr uint64_t GPS_TIMEOUT_MILLIS = 1000*5;


namespace Cohete {

    static TimerHandle_t xTimerRecalibrarMPU;
    static bool flag_timerRecalibrarMPU_disparado = false;

    // typedef void (* TimerCallbackFunction_t)( TimerHandle_t xTimer );
    void calibrar_mpu_callback(TimerHandle_t xTimer) {
        Sensors::getMPU6050().calibrar();
        flag_timerRecalibrarMPU_disparado = true;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    namespace Evento {

        bool gps_es_preciso(const data_all_t* datos_sensores) {
            return datos_sensores->gps_nro_satelites >= 5       // Mínimo 4 para 3D, 5 o 6 es más seguro
                && datos_sensores->gps_fix_type == 3            // Equivalente a 3D Fix en u-blox (fixType == 3)
                && datos_sensores->gps_gnss_fix_ok == true      // ¡CRÍTICO! El flag del módulo que confirma que el arreglo es válido
                && datos_sensores->gps_pdop <= 2.0;              // Dilución de precisión (pDOP * 0.01f) menor o igual a 2.0
            // && datos_sensores->gps_hacc <= 2500; // ¡EXTRA! Precisión horizontal (hAcc) menor a 2.5 metros (2500 mm)
        }

        bool en_codiciones_para_volar(data_all_t* datos_sensores) {
            return flag_timerRecalibrarMPU_disparado; // TODO: COMPLETAR CONDICIONES PARA VUELO
        }

        bool hay_boost(const data_all_t* datos_sensores) {
            // aceleracion >= 2 g por 0,15 segs
            // 2gs = 2 * 9.81m/s2
            if (SYSTEM.timestamp_micros_inicio_pico_g <= micros() && micros() <= 150) {
                if (datos_sensores->aceleracion_z_m_s2 >= 2*A_GRAV) {
                    return true;
                }
            }
            return false;
        }

    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void f_st_init(data_all_t* datos_sensores) {
        SerialPrint::msg("MdE Entrando -> [INIT]");

        // Esperar 5 minutos para que la mpu entre en calor, luego calibrarla.
        xTimerRecalibrarMPU =
            xTimerCreate(
                "Recalibrar",
                pdMS_TO_TICKS(1000*60*5),
                pdFALSE, // one shot timer
                nullptr,
                calibrar_mpu_callback
            );
        if( xTimerRecalibrarMPU != NULL ) {
            /* Iniciamos el temporizador con un tiempo de bloqueo (block time) de 0 */
            xTimerStart( xTimerRecalibrarMPU, 0 );
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
        if (es_entrada_a_estado()) {
            if (GSE::estado_conexion_gse() == ROCKET_DISCONNECTED || GSE::estado_conexion_gse() == ROCKET_INIT) {
                timestamp_millis_inicio_timeout = millis();
            }

        }
        else if (GSE::estado_conexion_gse() == ROCKET_WAITING_PONG) {
            // TODO: Qué hacemos mientras cohete espera conexión con GSE ?
            if ((millis() - timestamp_millis_inicio_timeout) >= CONEXION_GSE_TIMEOUT_MILLIS) { // TODO: Revisar si el timeout del gse es conveniente
                transicion_error(ERR_TIMEOUT_CONEXION_GSE, datos_sensores);
            }
        }
        else if (GSE::estado_conexion_gse() == ROCKET_CONNECTED) {
            transicionar_hacia(ST_ESPERA_GPS_PRECISO);
        }
    }

    void f_st_espera_gps_preciso(data_all_t* datos_sensores) {
        static uint64_t timestamp_millis_inicio_timeout;

        if (es_entrada_a_estado()) {
            timestamp_millis_inicio_timeout = millis();
        }
        else if ((millis() - timestamp_millis_inicio_timeout) >= GPS_TIMEOUT_MILLIS) {
            transicion_error(ERR_GPS_TIMEOUT, datos_sensores);
        }
        if (datos_sensores->gps_nro_satelites < 4) {
            Serial.printf("[GPS] Esperando satélites. Visibles: %d", datos_sensores->gps_nro_satelites);
        }
        else if (Evento::gps_es_preciso(datos_sensores)) {
            SerialPrint::msg(" -> [GPS] Precisión de GPS asegurado.");

            transicionar_hacia(ST_ESPERA_IGNICION);
        }
    }

    void f_st_espera_ignicion(data_all_t* datos_sensores) {
        // Aquí el cohete está pasivo en la rampa. Ignición es externa.
        // Lógica del filtro anti-zarandeo:
        // 1. Transformar aceleración a vector inercial.
        // 2. if (aceleracion_vertical >= 2.0g) {
        //      iniciar temporizador (timestamp_inicio_pico_g)
        //    }
        // 3. if (tiempo_con_2g >= 150ms AND delta_altura > 4.0m) {
        //      Transición a ST_PROPULSION.
        //    }

        if(Evento::en_codiciones_para_volar(datos_sensores)) {
            // if led no encendido: encenderlo para señalizar que ya podemos volar.
            if (Evento::hay_boost(datos_sensores)) {
                SYSTEM.timestamp_micros_inicio_pico_g = micros();
                transicionar_hacia(ST_BOOST);
            }
        }
        else if (Evento::hay_boost(datos_sensores)) {
            transicion_error(ERR_DESPEGUE_PROHIBIDO, datos_sensores);
        }
    }

    void f_st_boost(data_all_t* datos_sensores) {
        // como son static se crean una sola vez
        static float altura_entrada_st_boost;
        static float aceleracion_z_entrada_st_boost;
        static float velocidad_z_entrada_st_boost;

        if (es_entrada_a_estado()) {
            altura_entrada_st_boost = datos_sensores->altura_m;
            aceleracion_z_entrada_st_boost = datos_sensores->aceleracion_z_m_s2;
            velocidad_z_entrada_st_boost = datos_sensores->velocidad_z_m_s;
        }
        else if ((datos_sensores->altura_m - altura_entrada_st_boost) < 10) { // TODO: DEFINIR BIEN LA CONDICION de DIFF ALTURAS, PARA RESPALDAR QUE ENTRAMOS A BOOST REALMENTE.
            // si la diferencia no es considerable como para respaldar que estamos definitivamente en modo BOOST...
            transicion_error(ERR_DESPEGUE_FALSO_ZARANDEO, datos_sensores);
        }

        // Cuando la aceleración vertical decaiga bruscamente (Burn-out / Fin de combustión)
        // if (datos_sensores-> altura_m < altura_entrada_st_boost) { --> esto no determina comienzo de fase balistica, es parte de, sí, pero no determina. --> la fase balistica comienza con una velocidad hacia arriba que va desacelerandose por la gravedad --> al comienzo, la altura sigue incrementandose, aunque a tasas cada vez menores, hasta que empezamos a tener velocidad hacia abajo.
        if ((aceleracion_z_entrada_st_boost - datos_sensores->aceleracion_z_m_s2) > 0) { // Detectamos que hubo una DESaceleracion.
            // comparamos velocidad en el tiempo
            // TODO: Debemos comprobar que realmente hubo una DESaceleracion
            // si hubo desaceleracion, deberiamos ver una velocidad cada vez menor.
            // --> esto no nos confirma nada, ya que la velocidad la calculamos a partir de la aceleracion.
            // Deberiamos comprobarlo con otros datos, quizas el GPS sea nuestro mejor aliado en este problema.
            if ((velocidad_z_entrada_st_boost - datos_sensores->velocidad_z_m_s) > 0) {
                SYSTEM.masa_cohete_kg -= PESO_KG_COMBUSTIBLE; // TODO: asumimos que el combustible se consumio completamente ?
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


        if (SYSTEM.contexto_fisico.altura_max_historica < datos_sensores->altura_m) {
            // encontramos nueva altura historica
            SYSTEM.contexto_fisico.altura_max_historica = datos_sensores->altura_m;
        }
        // chequeando para cambiar a ST_APOGEO
        // asumimos que ya pasamos EL instante del apogeo, y estariamos por ende
        // con velocidad hacia abajo
        // con aceleracion hacia abajo (constante como siempre, la de gravedad)
        else if (datos_sensores->velocidad_z_m_s <= 0 // se mueve hacia abajo
            && datos_sensores->aceleracion_z_m_s2 < -A_GRAV + 2 // +2 para tener en cuenta errores
            )
        {
            // aproxima(datos_sensores->altura_m, ALTURA_M_MAX, 5) // TODO: Ver que cosas interesantes se puede hacer con esto.
            // transicionamos hacia ST_APOGEO --> abrimos drogue
            SYSTEM.timestamp_micros_apertura_drogue = micros();
            Actuators::getPyroDrogue().armar();
            Actuators::getPyroDrogue().disparar();
            transicionar_hacia(ST_APOGEO);
        }
    }

    void f_st_apogeo(data_all_t* datos_sensores) {
        // Lógica:
        // 1. Enviar señal de STOP a la cámara por pin/UART.
        // 2. Ignición pirotécnica del Drogue.
        // 4. Transición inmediata a ST_DESCENSO_EVALUACION.

        // chequeamos que el paracaidas drogue realmente se desplegó
        // medimos continuidad del pyro
        // medimos que la velocidad sea constante (con cierto ruido que debemos ignorar) gracias al drogue haciendo friccion con el aire.
        if (es_entrada_a_estado()) {

        }

        // chequear que realmente llegamos a apogeo
        // comprobar que altura paso por un punto mas alto y descendio inmediatamente
        if (datos_sensores->velocidad_z_m_s <= 0) { // esta cayendo
            if (datos_sensores->altura_m < SYSTEM.contexto_fisico.altura_max_historica) {
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
        //      SerialPrint::msg(" -> [NOMINAL] Paracaídas Principal desplegado.");
        //      Transición a ST_ATERRIZAJE (o estado intermedio de espera).
        // }
    }

    void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores) {
        SerialPrint::msg(" -> [EMERGENCIA] Drogue fallido. Disparando Principal de inmediato!");
        // Lógica:
        // 1. Disparo inmediato del paracaídas principal.
        // 2. Transición a ST_ATERRIZAJE (esperando el suelo).
    }

    void f_st_ejecutar_panico_flash_dump(data_all_t* datos_sensores) {
        SerialPrint::msg(" -> [FATAL] Caída libre detectada. Volcando RAM a Flash!");
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

        SerialPrint::err("[ST_ERROR]");
        SerialPrint::err("ERROR DESCONOCIDO SIN MANEJAR.");
        SerialPrint::plot("SYSTEM.error", SYSTEM.error);

    }

}
