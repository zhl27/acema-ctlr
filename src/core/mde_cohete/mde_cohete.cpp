//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"


system_data_t COHETE = {
    .estado = ST_INIT,
    .error = ERR_NINGUNO,
    .entrando_estado = false,
    // .es_estado_salida = false,

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
    // f_st_warmup_mpu,
    f_st_espera_conexion_gse,
    f_st_espera_gps_preciso,
    f_st_espera_boost,
    f_st_boost,
    f_st_fase_balistica,
    f_st_apogeo,
    f_st_evaluar_supervivencia_drogue, // TODO: REVISAR A PARTIR DE ACÁ
    f_st_descenso_controlado_drogue,
    f_st_desplegar_principal_emergencia,
    f_st_ejecutar_panico_flash_dump,
    f_st_transmitir_baliza_aterrizaje,
    f_st_procesar_falla_de_sistema
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
    if (COHETE.estado > ST_NULL) {
        transicion_error(ERR_ESTADO_INVALIDO, datos_sensores);
    }

    // actualizar datos de COHETE con datos nuevos de los sensores

    // if (SISTEMA.baro.presionBar > SISTEMA.limPresion.max || SISTEMA.flags.emergencia == 1) {
    //     transicionError(self, datosEnsayo);
    // }

    MDE_COHETE[COHETE.estado](datos_sensores); // Ejecuta la función que corresponde al estado actual
}

void transicion_error(const cod_error_t &error, data_all_t* datos_sensores) {
    COHETE.error=error;
    transicionar_hacia(ST_ERROR);

    // Acción de seguridad
    // paramos todos los timers ?
    // TODO: Definir qué se hace en cada tipo de error.
    // definimos en alguna función de estado el error que surge tal que así:
    // COHETE.error = ERR_TRAYECTORIA_PELIGROSA;
}

void transicionar_hacia(estado_vuelo_t nuevo_estado) {
    if (nuevo_estado > ST_NULL) {
        Serial.printf("Codigo enum %d Estado es inválido", nuevo_estado);
        return;
    }
    COHETE.estado = nuevo_estado;
    COHETE.entrando_estado = true;
    COHETE.timestamp_micros_entrada_estado = micros(); // grabamos este instante de transicion en el que entramos a un nuevo estado
    Serial.printf("[MdE] Entrando -> %s", estado_vuelo_string[nuevo_estado]);
}

bool es_entrada_a_estado() {
    const bool aux = COHETE.entrando_estado;
    COHETE.entrando_estado = false;
    return aux;
}
