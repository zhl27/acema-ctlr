//
// Created by zhl on 6/3/26.
//

#include "acciones.h"


// --- Hardware Mock Implementations ---
void configurar_sensores() { Serial.println(" -> Sensores configurados."); }
void configurar_actuadores() { Serial.println(" -> Actuadores configurados."); }
void conectar_GSE() { static uint long t = 0; if(millis() - t > 2000){ Serial.println(" -> Intentando conectar con GSE..."); t = millis(); }}
void get_cmd_GSE() { /* Reads incoming LoRa commands */ }
void init_rutina_propulsion() { Serial.println(" -> ¡Rutina de propulsión encendida!"); }
void apagar_motor() { Serial.println(" -> Motor APAGADO."); }
void init_rutina_frenado_aerodinamico() { Serial.println(" -> Frenado aerodinámico activo."); }
void desplegar_droge() { Serial.println(" -> Paracaídas DROGE desplegado."); }
bool droge_estabilizado() { return true; }
void desplegar_paracaidas_principal() { Serial.println(" -> ¡PARACAÍDAS PRINCIPAL DESPLEGADO!"); }