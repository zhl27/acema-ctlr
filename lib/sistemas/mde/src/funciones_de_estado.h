//
// Created by zhl on 6/11/26.
//

#ifndef ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
#define ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
#include "data.h"


typedef void (*f_st_t)(data_all_t* datos_sensores); // Punteros a funciones de estado.

// --- Hardware / Action Prototypes ---
void f_st_configurar_sensores();
void f_st_configurar_actuadores();
void f_st_conectar_GSE();
void f_st_get_cmd_GSE();
void f_st_init_rutina_propulsion();
void f_st_apagar_motor();
void f_st_init_rutina_frenado_aerodinamico();
void f_st_desplegar_droge();
bool st_droge_estabilizado();
void f_st_desplegar_paracaidas_principal();


#endif //ACEMA_CTLR_FUNCIONES_DE_ESTADO_H
