//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

void mde_cohete_actualizar(data_all_t* datos_sensores) {
    if (COHETE.estado < ST_NULL) {
        // if (SISTEMA.baro.presionBar > SISTEMA.limPresion.max || SISTEMA.flags.emergencia == 1) {
        //     transicionError(self, datosEnsayo);
        // }
        MDE_COHETE[COHETE.estado](datos_sensores); // Ejecuta la función que corresponde al estado actual
    } else {
        manejar_error(datos_sensores);
    }
}

void manejar_error(data_all_t* datos_sensores) {
    COHETE.estado = estado_vuelo_t::ST_ERROR;
    // Acción de seguridad
    // TODO: Definir qué se hace en cada tipo de error.
    // definimos en alguna función de estado el error que surge tal que así:
        // COHETE.error = ERR_TRAYECTORIA_PELIGROSA;

}



system_data_t COHETE = {
    .estado = ST_INIT,
    .error = ERR_NINGUNO,
    .vuelo_en_silencio_radio = false,
    .usar_uart_camara_como_sd = false,
    .timestamp_entrada_estado = 0,
    .timestamp_inicio_pico_g = 0,
    .timestamp_apertura_drogue = 0,
    .contexto_fisico = {
        .cota_suelo_rampa = 0.0f,
        .altura_actual_filtrada = 0.0f,
        .altura_max_historica = 0.0f,
        .temperatura_ambiente = 15.0f, // Valor por defecto sensato
        .densidad_aire = 1.225f,       // Densidad a nivel del mar estándar
        .acel_world = {0.0f, 0.0f, 0.0f},
        .vel_world = {0.0f, 0.0f, 0.0f},
        .pos_world = {0.0f, 0.0f, 0.0f},
        .cuaternion_actitud = {1.0f, 0.0f, 0.0f, 0.0f} // Identidad
    }
};

const f_st_t MDE_COHETE[] = {
    f_st_inicializar_sistema,
    f_st_ejecutar_warmup_mpu,
    f_st_buscar_enlace_gse_timeout,
    f_st_esperar_gps_fix,
    f_st_esperar_impulso,
    f_st_gestionar_ascenso_propulsado,
    f_st_gestionar_vuelo_balistico_y_freno,
    f_st_disparar_apogeo_y_detener_cam,
    f_st_evaluar_supervivencia_drogue,
    f_st_descenso_controlado_drogue,
    f_st_desplegar_principal_emergencia,
    f_st_ejecutar_panico_flash_dump,
    f_st_transmitir_baliza_aterrizaje,
    f_st_procesar_falla_de_sistema
};
