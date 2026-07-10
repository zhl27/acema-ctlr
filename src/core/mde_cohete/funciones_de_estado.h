//
// Created by zhl on 6/11/26.
//

#ifndef ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
#define ACEMA_CTLR_FUNCIONES_DE_ESTADO_H

#include "data.h"
#include "mBuzzer.h"

extern mBuzzer buzzer;

namespace Cohete {

    // Punteros a funciones de estado. Reciben el array de datos de los sensores.
    typedef void (*f_st_t)(data_all_t* datos_sensores);

    // --- Prototipos de la Máquina de Estados de Vuelo ---
    void f_st_init(data_all_t* datos_sensores);
    void f_st_warmup_mpu(data_all_t* datos_sensores);
    void f_st_espera_conexion_gse(data_all_t* datos_sensores);
    void f_st_espera_gps_preciso(data_all_t* datos_sensores);
    void f_st_espera_ignicion(data_all_t* datos_sensores);
    void f_st_boost(data_all_t* datos_sensores);
    void f_st_fase_balistica(data_all_t* datos_sensores);
    void f_st_apogeo(data_all_t* datos_sensores);
    void f_st_evaluar_supervivencia_drogue(data_all_t* datos_sensores);
    void f_st_descenso_controlado_drogue(data_all_t* datos_sensores);
    void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores);
    void f_st_ejecutar_panico_flash_dump(data_all_t* datos_sensores);
    void f_st_aterrizaje(data_all_t* datos_sensores);
    void f_st_error(data_all_t* datos_sensores);

}

/// UTILS

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


#endif //ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
