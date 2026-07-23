//
// Created by zhl on 6/11/26.
//

#ifndef ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
#define ACEMA_CTLR_FUNCIONES_DE_ESTADO_H

#include "data.h"
#include "mBuzzer.h"


namespace Cohete {

    // Punteros a funciones de estado. Reciben el array de datos de los sensores y un clock.
    typedef void (*f_st_t)(data_all_t* datos_sensores, uint32_t ms_en_estado);
    // Firma actualizada de las funciones de estado
    //using EstadoFunc = void (*)(data_all_t* datos_sensores, uint32_t ms_en_estado);

    // --- Prototipos de la Máquina de Estados de Vuelo ---
    void f_st_init(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_espera_conexion_gse(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_espera_gps_preciso(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_espera_ignicion(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_boost(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_fase_balistica(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_drogue_desplegado(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_pcaidas_ppal_desplegado(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_caida_catastrofica(data_all_t* datos_sensores, uint32_t ms_en_estado);
    void f_st_aterrizado(data_all_t* datos_sensores, uint32_t ms_en_estado);

}



#endif //ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
