//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

#include "SerialPrint.h"

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
            .flujos = {
                .Sensors_a_StateMachine_enabled = true,
                .Sensors_a_Flash_enabled = true,
                .Sensors_a_Lora_enabled = true
            }
        },

        .timestamp_micros_entrada_estado = 0,
        .timestamp_micros_inicio_pico_g = 0,
        .timestamp_micros_apertura_drogue = 0,

        .contexto_fisico = {
            .cota_suelo_rampa = 0.0f,
            .altura_actual = 0.0f,
            .altura_max_historica = 0.0f,
            .acel_global = {0.0f, 0.0f, 0.0f},
            .vel_global = {0.0f, 0.0f, 0.0f},
            .pos_global = {0.0f, 0.0f, 0.0f},
            .cuaternion_actitud = {1.0f, 0.0f, 0.0f, 0.0f} // Identidad
        }
    };

    const f_st_t MDE_COHETE[] = {
        f_st_init,
        f_st_espera_conexion_gse,
        f_st_espera_gps_preciso,
        f_st_espera_ignicion,
        f_st_boost,
        f_st_fase_balistica,
        f_st_apogeo,
        f_st_evaluar_supervivencia_drogue, // TODO: REVISAR A PARTIR DE ACÁ
        f_st_descenso_controlado_drogue,
        f_st_desplegar_principal_emergencia,
        f_st_ejecutar_panico_flash_dump,
        f_st_aterrizaje,
        f_st_error,
    };

    // correlativo a estado_vuelo_t --> el orden importa
    const char* estado_vuelo_string[] = {
        "ST_INIT",
        "ST_ESPERA_CONEXION_GSE",
        "ST_ESPERA_GPS_PRECISO",
        "ST_ESPERA_IGNICION",
        "ST_BOOST",
        "ST_FASE_BALISTICA",
        "ST_APOGEO",
        "ST_DESCENSO_EVALUACION",
        "ST_DESCENSO_NOMINAL",
        "ST_DESCENSO_EMERGENCIA",
        "ST_CAIDA_CATASTROFICA",
        "ST_ATERRIZAJE",
        "ST_ERROR",
        "ST_NULL"
    };

    void mde_cohete_actualizar(data_all_t* datos_sensores) {
        if (SYSTEM.estado > ST_NULL) {
            transicion_error(ERR_ESTADO_INVALIDO, datos_sensores);
        }

        // actualizar datos de COHETE con datos nuevos de los sensores

        // if (SISTEMA.baro.presionBar > SISTEMA.limPresion.max || SISTEMA.flags.emergencia == 1) {
        //     transicionError(self, datosEnsayo);
        // }

        MDE_COHETE[SYSTEM.estado](datos_sensores); // Ejecuta la función que corresponde al estado actual
    }

    // void pausar_proceso(TaskHandle_t proceso) {
    //     vTaskSuspend(proceso);
    //     SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false; // TODO: PENSAR SOLUCION MEJOR PARA MANEJAR LOS FLUJOS. QUIZAS UNA LISTA INDEXADA.
    // }

    void transicion_error(const cod_error_t error, data_all_t *datos_sensores) {
        SYSTEM.error=error;

        switch (error) {
            case ERR_TIMEOUT_CONEXION_GSE:
                // matamos el proceso GSE asi no nos gasta recursos del cohete, o bajamos su frecuencia.
                vTaskSuspend(SYSTEM.procesos.xTaskLoraHandle);
                SYSTEM.procesos.flujos.Sensors_a_Lora_enabled = false;
                SerialPrint::msg("Suspendido el Task Lora.");
                // continuamos con la siguiente etapa.
                transicionar_hacia(ST_ESPERA_GPS_PRECISO);
                return;
            case ERR_GPS_TIMEOUT:
                // TODO: QUÉ HACEMOS SI SE DA EL TIMEOUT DEL GPS ?
                SerialPrint::msg("Timeout de GPS !!!");
                SerialPrint::msg("Seguimos con el siguiente estado de la MdE.");
                transicionar_hacia(ST_ESPERA_IGNICION);
                return;
            case ERR_NINGUNO:
                break;
            case ERR_MPU_CALIBRACION_FALLIDA:
                break;
            case ERR_DESPEGUE_FALSO_ZARANDEO:
                break;
            case ERR_DESPEGUE_PROHIBIDO:
                break;
            case ERR_TRAYECTORIA_NO_VERTICAL:
                break;
            case ERR_DROGUE_DESGARRO:
                break;
            case ERR_FRENADO_AERO_ATASCADO:
                break;
            case ERR_ESTADO_INVALIDO:
                break;
            case ERR_DESCONOCIDO:
                break;
        }

        // en caso de no ser ninguno de los anteriores
        transicionar_hacia(ST_ERROR);

        // Acción de seguridad
        // paramos todos los timers ?
        // TODO: Definir qué se hace en cada tipo de error.
        // en funcion de estado f_st_error, chequear por cada error
        // if (COHETE.error == ERR_TRAYECTORIA_PELIGROSA)
        //
    }

    void transicionar_hacia(const estado_t nuevo_estado) {
        if (nuevo_estado > ST_NULL) {
            // transicion_error(ERR_ESTADO_INVALIDO, nullptr);
            Serial.printf("Codigo enum %d es Estado inválido", nuevo_estado);
            return;
        }
        SYSTEM.estado = nuevo_estado;
        SYSTEM.entrando_estado = true;
        SYSTEM.timestamp_micros_entrada_estado = micros(); // grabamos este instante de transicion en el que entramos a un nuevo estado
        Serial.printf("[MdE] Entrando -> %s\n", estado_vuelo_string[nuevo_estado]);
    }

    bool es_entrada_a_estado() {
        const bool aux = SYSTEM.entrando_estado;
        SYSTEM.entrando_estado = false;
        return aux;
    }

}
