//
// Created by zhl on 6/11/26.
//

#ifndef ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
#define ACEMA_CTLR_FUNCIONES_DE_ESTADO_H

#include "data.h"

// Punteros a funciones de estado. Reciben el array de datos de los sensores.
typedef void (*f_st_t)(data_all_t* datos_sensores);

// --- Prototipos de la Máquina de Estados de Vuelo ---
void f_st_inicializar_sistema(data_all_t* datos_sensores);
void f_st_ejecutar_warmup_mpu(data_all_t* datos_sensores);
void f_st_buscar_enlace_gse_timeout(data_all_t* datos_sensores);
void f_st_esperar_gps_fix(data_all_t* datos_sensores);
void f_st_esperar_impulso(data_all_t* datos_sensores);
void f_st_gestionar_ascenso_propulsado(data_all_t* datos_sensores);
void f_st_gestionar_vuelo_balistico_y_freno(data_all_t* datos_sensores);
void f_st_disparar_apogeo_y_detener_cam(data_all_t* datos_sensores);
void f_st_evaluar_supervivencia_drogue(data_all_t* datos_sensores);
void f_st_descenso_controlado_drogue(data_all_t* datos_sensores);
void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores);
void f_st_ejecutar_panico_flash_dump(data_all_t* datos_sensores);
void f_st_transmitir_baliza_aterrizaje(data_all_t* datos_sensores);
void f_st_procesar_falla_de_sistema(data_all_t* datos_sensores);

#endif //ACEMA_CTLR_FUNCIONES_DE_ESTADO_H