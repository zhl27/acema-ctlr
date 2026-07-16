//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

#include "esp_log.h"



namespace Cohete {

    system_data_t SYSTEM = {
        .estado = ST_INIT,
        .error = ERR_NINGUNO,
        .entrando_estado = false,
        // .es_estado_salida = false,

        .procesos ={
            .xTaskReadSensorsHandle = NULL,
            .xTaskStateMachineHandle = NULL,
            .xTaskFlashHandle = NULL,
            .xTaskLoraHandle = NULL,
            .xTaskDataFilterHandle = NULL,
            .flujos = {
                .Sensors_a_StateMachine_enabled = true,
                .Sensors_a_Flash_enabled = true,
                .Sensors_a_Lora_enabled = true
            }
        },

        .timestamp_micros_entrada_estado = 0,
        .timestamp_millis_inicio_pico_g = 0,
        .timestamp_micros_apertura_drogue = 0,

        .ctx_fisico = {
            .altura_m_max_historica = 0.0f,
            .masa_cohete_kg = 0.0f, // TODO: masa_cohete_kg debe ser configurable a traves de comando desde GSE: "set_masa_cohete_kg" o similar
            .altitud_cero_pad_m = 0.0f // TODO: altitud_cero_pad_m debe ser configurable a traves de comando desde GSE: "tara_altitud_cero_pad_m" o similar
            // .acel_global = {0.0f, 0.0f, 0.0f},
            // .vel_global = {0.0f, 0.0f, 0.0f},
            // .pos_global = {0.0f, 0.0f, 0.0f},
            // .cuaternion_actitud = {1.0f, 0.0f, 0.0f, 0.0f} // Identidad
        }
    };

    // correlativo a estado_vuelo_t --> el orden importa
    const f_st_t MDE_COHETE[] = {
        f_st_init,
        f_st_espera_conexion_gse,
        f_st_espera_gps_preciso,
        f_st_espera_ignicion,
        f_st_boost,
        f_st_fase_balistica,
        f_st_despliegue_drogue,
        f_st_evaluar_supervivencia_drogue, // TODO: REVISAR A PARTIR DE ACÁ
        f_st_descenso_controlado_drogue,
        f_st_desplegar_principal_emergencia,
        f_st_ejecutar_panico_flash_dump,
        f_st_aterrizaje,
        f_st_error,
    };

    // correlativo a estado_vuelo_t --> el orden importa
    const char* estado_cohete_string[] = {
        "ST_INIT",
        "ST_ESPERA_CONEXION_GSE",
        "ST_ESPERA_GPS_PRECISO",
        "ST_ESPERA_IGNICION",
        "ST_BOOST",
        "ST_FASE_BALISTICA",
        "ST_DESPLIEGUE_DROGUE",
        "ST_DESCENSO_EVALUACION",
        "ST_DESCENSO_NOMINAL",
        "ST_DESCENSO_EMERGENCIA",
        "ST_CAIDA_CATASTROFICA",
        "ST_ATERRIZAJE",
        "ST_ERROR",
        "ST_NULL"
    };
    // const size_t TAG_MAX_LEN = TAG_BASE_LEN+22;
    // static char TAG[TAG_MAX_LEN];





    void mde_cohete_actualizar(data_all_t* datos_sensores) {
        if (SYSTEM.estado >= ST_NULL) {
            transicion_error(ERR_ESTADO_INVALIDO, datos_sensores);
            return;
        }

        // actualizar datos de COHETE con datos nuevos de los sensores

        MDE_COHETE[SYSTEM.estado](datos_sensores); // Ejecuta la función que corresponde al estado actual
    }

    // void pausar_proceso(TaskHandle_t proceso) {
    //     vTaskSuspend(proceso);
    //     SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false; // TODO: PENSAR SOLUCION MEJOR PARA MANEJAR LOS FLUJOS. QUIZAS UNA LISTA INDEXADA.
    // }

    void transicion_error(const error_cohete_t error, data_all_t *datos_sensores) {
        SYSTEM.error=error;

        switch (error) {
            case ERR_TIMEOUT_CONEXION_GSE:
                ESP_LOGE(TAG_BASE, "Timeout de conexión con GSE.");
                // matamos el proceso GSE asi no nos gasta recursos del cohete, o bajamos su frecuencia.
                vTaskSuspend(SYSTEM.procesos.xTaskLoraHandle);
                SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false;
                ESP_LOGI(TAG_BASE, "Suspendido el Task Lora, ya que no nos comunicaremos con la GSE.");
                // continuamos con la siguiente etapa.
                transicionar_hacia(ST_ESPERA_GPS_PRECISO);
                return;
            case ERR_GPS_TIMEOUT:
                // TODO: QUÉ HACEMOS SI SE DA EL TIMEOUT DEL GPS ?
                ESP_LOGE(TAG_BASE, "Timeout de GPS.");
                transicionar_hacia(ST_ESPERA_IGNICION);
                return;
            // case ERR_NINGUNO:
            //
            //     return;
            case ERR_MPU_CALIBRACION_FALLIDA:
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_MPU_CALIBRACION_FALLIDA");
                return;
            case ERR_DESPEGUE_FALSO_ZARANDEO:
                ESP_LOGE(TAG_BASE, "ERR_DESPEGUE_FALSO_ZARANDEO. Volvemos a ST_ESPERA_IGNICION.");
                SYSTEM.timestamp_millis_inicio_pico_g = 0;
                transicionar_hacia(ST_ESPERA_IGNICION);
                return;
            case ERR_DESPEGUE_PROHIBIDO:
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_DESPEGUE_PROHIBIDO");
                // TODO: Qué hacemos si realmente detectamos un despegue, pero el cohete no estaba en condiciones de volar? Pienso que: ya que esta en vuelo, mucho no podemos hacer al respecto, hay que continuar con lo que se tiene. Ver qué hacemos a partir de ahí.
                return;
            case ERR_TRAYECTORIA_NO_VERTICAL:
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_TRAYECTORIA_NO_VERTICAL");
                return;
            case ERR_DROGUE_NO_EFECTO:
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_DROGUE_NO_EFECTO");
                return;
            case ERR_FRENADO_AERO_ATASCADO:
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_FRENADO_AERO_ATASCADO");
                return;
            case ERR_ESTADO_INVALIDO:
                ESP_LOGE(TAG_BASE, "FALTA IMPLEMENTAR. ERR_ESTADO_INVALIDO");
                return;
            // case ERR_DESCONOCIDO:
            //     ESP_LOGE(TAG_BASE, "ERROR DESCONOCIDO.");
            //     return;
        }

        // en caso de no ser ninguno de los anteriores
        transicionar_hacia(ST_ERROR); // TODO: creo que no necesitamos un ST_ERROR, podemos usar esta función para manejar las cosas.

        // Acción de seguridad
        // paramos todos los timers ?
        // TODO: Definir qué se hace en cada tipo de error.
        // en funcion de estado f_st_error, chequear por cada error
        // if (COHETE.error == ERR_TRAYECTORIA_PELIGROSA)
        //
    }

    void transicionar_hacia(const estado_cohete_t nuevo_estado) {
        // TODO: Volver a visitar esta curiosidad. Ver "mde_cohete/include.h"
        // strlcpy(TAG, TAG_BASE, TAG_BASE_LEN);
        // strlcat(TAG, " - ", TAG_BASE_LEN+3);
        // strlcat(TAG, estado_cohete_string[nuevo_estado], TAG_BASE_LEN);
        if (nuevo_estado >= ST_NULL) {
            // transicion_error(ERR_ESTADO_INVALIDO, nullptr);
            ESP_LOGE(TAG_BASE, "Codigo enum %d es Estado inválido.", nuevo_estado);
            return;
        }
        SYSTEM.estado = nuevo_estado;
        SYSTEM.entrando_estado = true;
        SYSTEM.timestamp_micros_entrada_estado = micros(); // grabamos este instante de transicion en el que entramos a un nuevo estado
        ESP_LOGI(TAG_BASE, "Entrando a: %s", estado_cohete_string[nuevo_estado]);
    }

    bool entrando_a_estado() {
        const bool aux = SYSTEM.entrando_estado;
        SYSTEM.entrando_estado = false;
        return aux;
    }

}
