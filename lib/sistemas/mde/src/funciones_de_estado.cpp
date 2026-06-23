//
// Created by zhl on 6/11/26.
//

#include "funciones_de_estado.h"

// TODO: Definir correctamente la funcionalidad de cada función de estado, actualmente son solo placeholders para simular la lógica de la máquina de estados.

// --- Hardware Mock Implementations ---
void f_st_configurar_sensores() {
    SerialPrint::msg(" -> Sensores configurados.");
}

void f_st_configurar_actuadores() {
    SerialPrint::msg(" -> Actuadores configurados.");
}

void f_st_conectar_GSE() {
    static unsigned int long t = 0;
    if(millis() - t > 2000) {
        SerialPrint::msg(" -> Intentando conectar con GSE..."); t = millis();
    }
}

void f_st_get_cmd_GSE() {
    /* Reads incoming LoRa commands */
}

void f_st_init_rutina_propulsion() {
    SerialPrint::msg(" -> ¡Rutina de propulsión encendida!");
}

void f_st_apagar_motor() {
    SerialPrint::msg(" -> Motor APAGADO.");
}

void f_st_init_rutina_frenado_aerodinamico() {
    SerialPrint::msg(" -> Frenado aerodinámico activo.");
}

void f_st_desplegar_droge() {
    SerialPrint::msg(" -> Paracaídas DROGE desplegado.");
}

bool st_droge_estabilizado() {
    return true;
}

void f_st_desplegar_paracaidas_principal() {
    SerialPrint::msg(" -> ¡PARACAÍDAS PRINCIPAL DESPLEGADO!");
}