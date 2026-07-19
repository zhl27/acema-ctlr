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
constexpr float UMBRAL_ACEL_BOOST = 2 * A_GRAV;     // 2G (acorde a requerimientos)
constexpr int TIEMPO_MS_MIN_BOOST = 200;            // 0.2 segundos en milisegundos (acorde a requerimientos)
constexpr uint32_t CONEXION_GSE_TIMEOUT_MILLIS = 1000*60*2;
constexpr float ALTURA_M_MAX = 1000; // TODO: chequear ALTURA_M_MAX. Igual nos importa realmente este dato?
constexpr uint32_t GPS_TIMEOUT_MILLIS = 1000*5;
constexpr uint32_t TIEMPO_MILLIS_ESPERA_WARMUP_MPU = 1000*60*5;
constexpr int DIFF_ALTURA_M_APOGEO_CAIDA = 10;
constexpr int ALTITUD_DESPLIEGUE_PCAIDAS_PPAL = 250;



namespace Cohete {

    namespace Timers
    {
        static TimerHandle_t xTimerRecalibrarMPU;
        static bool flag_recalibrarMPU_disparado = false;

        // typedef void (* TimerCallbackFunction_t)( TimerHandle_t xTimer );
        void calibrar_mpu_callback(TimerHandle_t xTimer) {
            ESP_LOGI(TAG_BASE, "Temporizador xTimerRecalibrarMPU disparado!");
            int res = Sensors::getMPU6050().calibrar();
            if (res==0) {
                flag_recalibrarMPU_disparado = true;
                ESP_LOGI(TAG_BASE, "Calibración de MPU6050 exitosa.");
            }
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    namespace Eventos {
        // val0 --> valor de estudio
        // val1 --> valor objetivo
        // delta --> margen
        inline bool aproxima(const float val0, const float val1, const float delta) {
            return fabsf(val0 - val1) <= delta; // implementación que aprovecha la FPU de la ESP32
            // return (val0 <= val1 + delta && val0 >= val1 - delta);
        }
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

            if (cumple) {
                if (ticks_cumple_condiciones < 255) ticks_cumple_condiciones++;
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

        bool en_codiciones_para_volar(data_all_t* datos_sensores) {
            return Timers::flag_recalibrarMPU_disparado; // este flag nos permite asumir que los datos MPU son aceptables
            // TODO: COMPLETAR CONDICIONES PARA VUELO.
            // queremos sí o sí el GPS para el vuelo?
        }

        bool hay_boost(data_all_t *datos_sensores) { // TODO: Completar lógica de Boost
            // aceleracion >= 2 g por 0,2 segs
            if (datos_sensores->aceleracion_z_m_s2 >= UMBRAL_ACEL_BOOST) { // detectamos un supuesto boost, chequeamos...
                if (SYSTEM.timestamp_millis_inicio_pico_g == 0) { // que sea 0 significa que nunca antes habiamos detectado inicio de boost --> imposible que tengamos boost en micros()==0
                    SYSTEM.timestamp_millis_inicio_pico_g = millis(); // detectamos un pico por primera vez y guardamos timestamp
                }
                else if (SYSTEM.timestamp_millis_inicio_pico_g <= millis() && millis() - SYSTEM.timestamp_millis_inicio_pico_g >= TIEMPO_MS_MIN_BOOST) {
                    if (datos_sensores->altitud_filtrada_m > SYSTEM.ctx_fisico.altitud_m_pad + 4.0f) { // TODO: Revisar este 4
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
            timestamp_millis_inicio_timeout = millis();
            ESP_LOGI(TAG_BASE, " -> [CONEXIÓN GSE] Esperando conexión con GSE...");
        }

        // TODO: Qué hacemos mientras cohete espera conexión con GSE ?
        else if ((millis() - timestamp_millis_inicio_timeout) >= CONEXION_GSE_TIMEOUT_MILLIS) { // TODO: Revisar si el timeout del gse es conveniente
            // transicion_error(ERR_TIMEOUT_CONEXION_GSE, datos_sensores); // TODO: empiezo a considerar que la función "transicion_error" genera un nivel de indirección innecesario. Se podría poner la lógica del error acá mismo.
            ESP_LOGE(TAG_BASE, "Timeout de conexión con GSE.");
            // matamos el proceso GSE asi no nos gasta recursos del cohete, o bajamos su frecuencia.
            vTaskSuspend(SYSTEM.procesos.xTaskLoraHandle);
            SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false;
            ESP_LOGI(TAG_BASE, "Suspendido el Task Lora, ya que no nos comunicaremos con la GSE.");
            // continuamos con la siguiente etapa.
            transicionar_hacia(ST_ESPERA_GPS_PRECISO);
        }

        if (GSE::estado_conexion_gse() == ROCKET_CONNECTED) {
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
            // transicion_error(ERR_GPS_TIMEOUT, datos_sensores);
            ESP_LOGE(TAG_BASE, "Timeout de GPS.");
            // TODO: QUÉ HACEMOS SI SE DA EL TIMEOUT DEL GPS ?
            transicionar_hacia(ST_ESPERA_IGNICION);
        }
        if (datos_sensores->gps_nro_satelites < 4) {
            if (millis() % 200 == 0)
                ESP_LOGI(TAG_BASE, " -> [GPS] Esperando satélites. Visibles: %d", datos_sensores->gps_nro_satelites);
        }
        else if (Eventos::gps_es_preciso(datos_sensores) || SYSTEM.gse_configs.gps_override_skip) {
            ESP_LOGI(TAG_BASE, " -> [GPS] Precisión de GPS asegurado!");
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

        if (entrando_a_estado()) {
            ESP_LOGI(TAG_BASE, "[ESPERA IGNICION] Esperando condiciones necesarias para el vuelo...");
        }

        static unsigned long last_millis = 0;
        if(Eventos::en_codiciones_para_volar(datos_sensores)) {
            // if led no encendido: encenderlo para señalizar que ya podemos volar.
            if (millis() - last_millis >= 3000) { // hacemos un beep cada 3 segundos
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
            // transicion_error(ERR_DESPEGUE_PROHIBIDO, datos_sensores);
            ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_DESPEGUE_PROHIBIDO");
            // TODO: Qué hacemos si realmente detectamos un despegue, pero el cohete no estaba en condiciones de volar? Pienso que: ya que esta en vuelo, mucho no podemos hacer al respecto, hay que continuar con lo que se tiene. Ver qué hacemos a partir de ahí.
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
        // TODO: Revisar esta lógica
        else if ((datos_sensores->altitud_filtrada_m - altura_entrada_st_boost) >= 2) {
            // si la diferencia no es considerable como para respaldar que estamos definitivamente en modo BOOST...
            // transicion_error(ERR_DESPEGUE_FALSO_ZARANDEO, datos_sensores);
            ESP_LOGE(TAG_BASE, "ERR_DESPEGUE_FALSO_ZARANDEO. Volvemos a ST_ESPERA_IGNICION.");
            SYSTEM.timestamp_millis_inicio_pico_g = 0;
            transicionar_hacia(ST_ESPERA_IGNICION);
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
                SYSTEM.ctx_fisico.masa_g_cohete -= SYSTEM.ctx_fisico.masa_g_combustible; // TODO: asumimos que el combustible se consumio completamente ?
                transicionar_hacia(ST_FASE_BALISTICA);
            }
        }
    }

    void f_st_fase_balistica(data_all_t* datos_sensores) {
        // ACTUALIZACIÓN DE ALTURA MÁXIMA (Con protección contra picos durante descenso)
        // Solo actualizamos el récord histórico si la velocidad aún es positiva o cercana a cero
        if (datos_sensores->vel_z_filtrada_m_s > -0.2f) {
            if (datos_sensores->altitud_filtrada_m > SYSTEM.ctx_fisico.altura_m_max_historica) {
                SYSTEM.ctx_fisico.altura_m_max_historica = datos_sensores->altitud_filtrada_m;
            }
        }

        // GATES DE DETECCIÓN DE APOGEO
        const bool vel_apogeo = (datos_sensores->vel_z_filtrada_m_s <= -0.3f);
        const bool caida_confirmada = (SYSTEM.ctx_fisico.altura_m_max_historica - datos_sensores->altitud_filtrada_m) >= DIFF_ALTURA_M_APOGEO_CAIDA; // diff de 10 metros según requerimientos.

        // En caída libre (sin empuje de motor), está bajo aceleracion gravitatoria pero afectada por el arrastre (drag).
        // Un margen de [-5.0, +2.0] m/s² como "G-Gate" para evitar disparos durante empuje o eyección de etapas.
        const bool g_gate_valido = (datos_sensores->aceleracion_z_m_s2 > -5.0f && datos_sensores->aceleracion_z_m_s2 < 2.0f);

        // FILTRO DE PERSISTENCIA (Debounce de Apogeo - Evita falsos positivos por ruido)
        // Requerimos que ambas condiciones (o caída confirmada) se mantengan por N ticks consecutivos (ej. 5 ticks = 33ms a 150Hz)
        static uint8_t ticks_apogeo_confirmado = 0;
        if ((vel_apogeo && g_gate_valido) || caida_confirmada) {
            if (ticks_apogeo_confirmado < 255) ticks_apogeo_confirmado++;
        } else {
            ticks_apogeo_confirmado = 0; // Se reinicia si fue un pico de ruido efímero
        }

        // ALCANZAMOS APOGEO O NO
        const bool apogeo_alcanzado_definitivamente = (ticks_apogeo_confirmado >= 5); // 5 ticks continuos confirman apogeo

        if (apogeo_alcanzado_definitivamente) {
            // Retraer el Airbrake antes de la eyección del drogue
            Actuators::getServo().sendAngulo(0.0f);

            // EVALUAR SI ENCENDER PYRO DE DROGUE O NO
            if (!SYSTEM.drogue_disparado) {
                if (!Actuators::getPyroDrogue().tieneContinuidad()) {
                    // TODO: AJUSTAR MEDIDAS DE SEGURIDAD DE PYRO DROGUE
                    Actuators::getPyroDrogue().armar();
                    Actuators::getPyroDrogue().disparar();
                    vTaskDelay(pdMS_TO_TICKS(2)); // TODO: Resolver tema del ringbuffer que tira error cuando usamos un delay en la MdE.
                    if (Actuators::getPyroDrogue().tieneContinuidad()) {
                        SYSTEM.drogue_disparado = true;
                        SYSTEM.timestamp_micros_apertura_drogue = micros();
                    }
                    else {
                        // ERROR CRÍTICO: Llegamos a apogeo, pero el MOSFETT está roto/desconectado. --> Significa que no podremos abrir drogue --> Intentamos abrir pcaidas ppal --> y si falla --> Prepararse para impactar --> Guardar en log Flash/SD y forzar transición tras un tiempo de espera de seguridad.
                        // transicion_error(ERR) // TODO: Falta implementar manejo de error
                    }
                }
            }
            else {
                // TODO: Chequear esto
                // El disparo ya se ordenó. Esperamos un tiempo físico de quemado (ej. 100ms)
                // antes de cambiar al estado de descenso, independientemente de la continuidad.
                if ((micros() - SYSTEM.timestamp_micros_apertura_drogue) >= 100000UL) {
                    Actuators::getPyroDrogue().desarmar(); // Cortar corriente al MOSFET por seguridad
                    transicionar_hacia(ST_DROGUE_DESPLEGADO);
                }
            }
        }
            // en caso de no haber alcanzado apogeo todavia
        else {
            // VUELO BALÍSTICO NOMINAL (Aún ascendiendo hacia el apogeo)
            // TODO: Reemplazar 0.0f por el controlador MPC
            float angulo_airbrake = 0.0f;

            // Medida de seguridad: Si la velocidad vertical ya es baja (< 15 m/s),
            // no extender frenos para no perder control aerodinámico cerca del apogeo.
            // TODO: También hay que detectar que no esté en coast --> creo que la condicion de la velocidad sirve, hay que chequear
            if (datos_sensores->vel_z_filtrada_m_s < 15.0f) {
                angulo_airbrake = 0.0f;
            }
            Actuators::getServo().sendAngulo(angulo_airbrake);
        }
    }

    void f_st_drogue_desplegado(data_all_t* datos_sensores) {
        // Lógica:
        // 1. Enviar señal de STOP a la cámara por pin/UART.
        // 2. Ignición pirotécnica del Drogue.
        // 4. Transición inmediata a ST_DESCENSO_EVALUACION.

        // Chequeamos que el paracaidas drogue realmente se desplegó
        // 1. Medimos continuidad del pyro
        // 2. Medimos que la velocidad sea constante (con cierto ruido que debemos ignorar) gracias al drogue haciendo friccion con el aire. --> equivalente seria medir que aceleracion aproxima a 0.
        if (entrando_a_estado()) {
            // vTaskDelay(pdMS_TO_TICKS(3)); // segun requerimientos --> preguntar: 3 milisegundos o 3 segundos

            if (Eventos::aproxima(datos_sensores->aceleracion_z_m_s2, 0.0f, 5.0f)) {
                ESP_LOGI(TAG_BASE, " -> [%s] Drogue desplegado correctamente.", estado_cohete_string[SYSTEM._estado]);
                transicionar_hacia(ST_PCAIDAS_PPAL_DESPLEGADO);
            }
        }
        // CHEQUEAR SI DEBEMOS DESPLEGAR PARACAIDAS PRINCIPAL O NO.
        // Utilizamos debounce para descartar picos y errores rápidamente.
        static uint8_t ticks_desplegamos_drogue = 0;
        if (datos_sensores->vel_z_filtrada_m_s <= 0) { // está cayendo
            if (SYSTEM.ctx_fisico.altitud_m_relativa_al_pad <= ALTITUD_DESPLIEGUE_PCAIDAS_PPAL) {
                if (ticks_desplegamos_drogue < 255) ticks_desplegamos_drogue++;
            }
            else {
                ticks_desplegamos_drogue = 0;
            }
        }
        if (ticks_desplegamos_drogue >= 10) { // se cumple la condicion 10 ticks consecutivos
            // TODO: pyro.armar();
            transicionar_hacia(ST_PCAIDAS_PPAL_DESPLEGADO);
        }

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

    void f_st_pcaidas_ppal_desplegado(data_all_t* datos_sensores) {
        // TODO: AHORA TENEMOS QUE CHEQUEAR QUE HAYA ATERRIZADO.
    }

    // void f_st_descenso_controlado_drogue(data_all_t* datos_sensores) {
    //     // Bajando con Drogue.
    //     // Lógica:
    //     // if (altura_actual_filtrada <= 250.0m) { // ¡Cuidado de chequear contra cota_suelo_rampa!
    //     //      Ignición pirotécnica Paracaídas Principal.
    //     //      // ESP_LOGI(TAG_STATE_MACHINE, " -> [NOMINAL] Paracaídas Principal desplegado.");
    //     //      Transición a ST_ATERRIZAJE (o estado intermedio de espera).
    //     // }
    // }

    void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores) {
        ESP_LOGI(TAG_BASE, " -> [EMERGENCIA] Drogue fallido. Disparando Principal de inmediato!");
        // Lógica:
        // 1. Disparo inmediato del paracaídas principal.
        // 2. Transición a ST_ATERRIZAJE (esperando el suelo).
    }

    void f_st_caida_catastrofica(data_all_t* datos_sensores) {
        ESP_LOGI(TAG_BASE, " -> [FATAL] Caída libre detectada. Volcando RAM a Flash!");
        // Lógica:
        // 1. Las tarjetas SD mecánicas pueden corromperse en impactos duros.
        // 2. Escribir el buffer circular de últimos 5 segundos en la Flash de la ESP32
        //    (NVS o SPIFFS/LittleFS) para análisis post-mortem.
        // 3. Este estado no tiene salida, es terminal antes del impacto.
    }

    void f_st_aterrizaje(data_all_t* datos_sensores) {
        // Lógica:
        // 1. Detectamos reposo en el suelo (posicion_world cerca de 0 relativa, acel == 0). --> datos_sensores->altitud_m es la altitud relativa al mundo
        // 2. Detener logs de alta frecuencia para salvar batería.
        // 3. Cerrar archivos en la SD (flush y close).
        // 4. Emitir un "beeeeep beeeeep" continuo y transmitir coordenadas Lat/Lon por LoRa cada X segundos.
    }

}
/**
 * Si el drogue no tiene continuidad, abrir el paracaidas
 * en el descenso
 * 100m/S menor use usa
 */
