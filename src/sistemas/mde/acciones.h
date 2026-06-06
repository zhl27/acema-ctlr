//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_ACCIONES_H
#define ACEMA_CTLR_ACCIONES_H


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


#endif //ACEMA_CTLR_ACCIONES_H
