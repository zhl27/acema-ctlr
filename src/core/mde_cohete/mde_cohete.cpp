//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

#include "esp_log.h"
#include "data.h"


namespace Cohete {

    system_data_t SYSTEM = {
        .estado = ST_INIT,
        .estado_anterior = ST_NULL,
        ._error = ERR_NINGUNO,
        ._entrando_estado = false,
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

        .gse_configs = {
            .gps_override_skip = false,
        },

        .timestamp_millis_entrada_estado = 0,
        ._timestamp_millis_inicio_pico_g = 0,
        .timestamp_micros_apertura_drogue = 0,

        .ctx_fisico = {
            .altura_m_max_historica = 0.0f,
            .masa_g_cohete = 0, // TODO: masa_cohete_kg debe ser configurable a traves de comando desde GSE: "set_masa_cohete_kg" o similar
            .masa_g_combustible = 0, // TODO: masa_combustible_kg debe ser configurable a traves de comando desde GSE: "set_masa_combustible_kg" o similar
            .altitud_m_pad = 0.0f, // TODO: altitud_m_pad toma el valor actual de la altitud_bmp --> cuando comando desde GSE: "tara_altitud" o similar
            .altitud_m_relativa_al_pad = 0.0f // se actualiza utilizando SYSTEM.ctx_fisico.altitud_m_cero_pad
        },

        .flags = {
            // .gse_conectado = false,
            .flash_log_borrado = false, // Se debe borrar el log de datos basura
            // .gps_preciso = false,
            .drogue_disparado = false,
            .paracaidas_principal_disparado = false,
            .emergencia_fatal = false,
            // .borrar_log = false,
            .volcar_ram_a_flash = false,
        },
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
        f_st_aterrizado,
        f_st_caida_catastrofica
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
        if (SYSTEM.estado >= ST_NULL) {
            return;
        }

        const uint32_t ahora_ms = millis();

#ifdef DEBUG_ESP32
        Serial.print(">estado_mde:");
        Serial.println(SYSTEM.estado);
#endif

        // Si hubo un cambio de estado, reiniciamos la referencia temporal
        if (SYSTEM.estado != SYSTEM.estado_anterior) {
            SYSTEM.timestamp_millis_entrada_estado = ahora_ms;
            SYSTEM.estado_anterior = SYSTEM.estado;
        }

        // Calculamos cuánto tiempo lleva el cohete en el estado actual
        const uint32_t ms_en_estado = ahora_ms - SYSTEM.timestamp_millis_entrada_estado;

        // Actualizar datos derivados de contexto físico
        SYSTEM.ctx_fisico.altitud_m_relativa_al_pad = 
            datos_sensores->altitud_filtrada_m_bmp - SYSTEM.ctx_fisico.altitud_m_pad;

        // Ejecutar el estado pasando los datos y el tiempo transcurrido
        MDE_COHETE[SYSTEM.estado](datos_sensores, ms_en_estado);
    }

    // void pausar_proceso(TaskHandle_t proceso) {
    //     vTaskSuspend(proceso);
    //     SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false; // TODO: PENSAR SOLUCION MEJOR PARA MANEJAR LOS FLUJOS. QUIZAS UNA LISTA INDEXADA.
    // }
}
