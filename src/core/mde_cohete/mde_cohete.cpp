//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

#include "esp_log.h"



namespace Cohete {

    system_data_t SYSTEM = {
        ._estado = ST_INIT,
        ._error = ERR_NINGUNO,
        ._entrando_estado = false,
        // .es_estado_salida = false,
        .drogue_disparado = false,

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
            .masa_g_cohete = 0, // TODO: masa_cohete_kg debe ser configurable a traves de comando desde GSE: "set_masa_cohete_kg" o similar
            .masa_g_combustible = 0, // TODO: masa_combustible_kg debe ser configurable a traves de comando desde GSE: "set_masa_combustible_kg" o similar
            .altitud_m_pad = 0.0f, // TODO: altitud_m_pad toma el valor actual de la altitud_bmp --> cuando comando desde GSE: "tara_altitud" o similar
            .altitud_m_relativa_al_pad = 0.0f // se actualiza utilizando SYSTEM.ctx_fisico.altitud_m_cero_pad
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
        f_st_drogue_desplegado,
        f_st_pcaidas_ppal_desplegado,
        f_st_caida_catastrofica,
        f_st_aterrizaje
    };

    // correlativo a estado_vuelo_t --> el orden importa
    const char* estado_cohete_string[] = {
        "ST_INIT",
        "ST_ESPERA_CONEXION_GSE",
        "ST_ESPERA_GPS_PRECISO",
        "ST_ESPERA_IGNICION",
        "ST_BOOST",
        "ST_FASE_BALISTICA",
        "ST_DROGUE_DESPLEGADO",
        "ST_PCAIDAS_PPAL_DESPLEGADO",
        "ST_CAIDA_CATASTROFICA",
        "ST_ATERRIZAJE",
        "ST_NULL"
    };


    void mde_cohete_actualizar(data_all_t* datos_sensores) {
        if (SYSTEM._estado >= ST_NULL) {
            transicion_error(ERR_ESTADO_INVALIDO, datos_sensores);
            return;
        }

        // actualizar datos de COHETE con datos nuevos de los sensores
        SYSTEM.ctx_fisico.altitud_m_relativa_al_pad = datos_sensores->altitud_filtrada_m - SYSTEM.ctx_fisico.altitud_m_pad; // TODO: chequear que altitud_filtrada_m sea altitud del bmp280 y que represente altitud al nivel del mar


        MDE_COHETE[SYSTEM._estado](datos_sensores); // Ejecuta la función que corresponde al estado actual
    }

    // void pausar_proceso(TaskHandle_t proceso) {
    //     vTaskSuspend(proceso);
    //     SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false; // TODO: PENSAR SOLUCION MEJOR PARA MANEJAR LOS FLUJOS. QUIZAS UNA LISTA INDEXADA.
    // }

    void transicion_error(const error_cohete_t error, data_all_t *datos_sensores) {
        if (error >= ERR_DESCONOCIDO) {
            ESP_LOGE(TAG_BASE, "ERROR DESCONOCIDO SIN MANEJAR.");
            ESP_LOGE(TAG_BASE, "SYSTEM.error=%d", SYSTEM._error);
            return;
        }

        SYSTEM._error=error;

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
        }

        // en caso de no ser ninguno de los anteriores
        // transicionar_hacia(ST_ERROR); // TODO: creo que no necesitamos un ST_ERROR, podemos usar esta función para manejar las cosas.

        // Acción de seguridad
        // paramos todos los timers ?
        // TODO: Definir qué se hace en cada tipo de error.
        // en funcion de estado f_st_error, chequear por cada error
        // if (COHETE.error == ERR_TRAYECTORIA_PELIGROSA)
        //
    }

    void transicionar_hacia(const estado_cohete_t nuevo_estado) {
        if (nuevo_estado >= ST_NULL) {
            // transicion_error(ERR_ESTADO_INVALIDO, nullptr);
            ESP_LOGE(TAG_BASE, "[%s] Codigo enum %d es Estado inválido.", estado_cohete_string[nuevo_estado], nuevo_estado);
            return; // el return hace que no se efectivice ninguna transicion
        }
        SYSTEM._estado = nuevo_estado;
        SYSTEM._entrando_estado = true;
        SYSTEM.timestamp_micros_entrada_estado = micros(); // grabamos este instante de transicion en el que entramos a un nuevo estado
        ESP_LOGI(TAG_BASE, "Entrando a: %s", estado_cohete_string[nuevo_estado]);
    }

    estado_cohete_t estado_actual() {
        return SYSTEM._estado;
    }

    bool entrando_a_estado() {
        const bool aux = SYSTEM._entrando_estado;
        SYSTEM._entrando_estado = false;
        return aux;
    }

}
