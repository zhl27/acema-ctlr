//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_ACCIONES_H
#define ACEMA_CTLR_ACCIONES_H


// --- Hardware / Action Prototypes ---
void configurar_sensores();
void configurar_actuadores();
void conectar_GSE();
void get_cmd_GSE();
void init_rutina_propulsion();
void apagar_motor();
void init_rutina_frenado_aerodinamico();
void desplegar_droge();
bool droge_estabilizado();
void desplegar_paracaidas_principal();


#endif //ACEMA_CTLR_ACCIONES_H
