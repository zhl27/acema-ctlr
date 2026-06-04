//
// Created by zhl on 6/3/26.
//

#include "handlers.h"


void handle_mde(){
    if( estado >=  N_EStado) {
        estado =flash.leerEstado ();
        if(){
            handle_error();
        }
    }
    mde_cohete[estado]();
}


void handle_error() {
    apagar_motor();
    lanzar_paracaida();
    estado = descenso;
}


// --- State Machine Core Execution ---
void runStateMachine() {
    switch (currentState) {

        case STATE_BUSCANDO_CONEXION:
            if (isEntry) {
                Serial.println("[STATE] Buscando Conexión");
                configurar_sensores();
                configurar_actuadores();
                isEntry = false;
            }
            conectar_GSE();
            break;

        case STATE_ESPERA_INICIO:
            if (isEntry) {
                Serial.println("[STATE] Espera Inicio");
                g_vel_transmision = LENTA;
                isEntry = false;
            }
            get_cmd_GSE();
            break;

        case STATE_PROPULSION:
            if (isEntry) {
                Serial.println("[STATE] Propulsion");
                propSubState = SUB_PRE_APERTURA_OX;
                Serial.println("  -> Substate: Pre_apertura_ox");
                isEntry = false;
            }

            // Substate Machine Logic
            if (propSubState == SUB_PRE_APERTURA_OX) {
                // Perform oxygen valve tasks...
                propSubState = SUB_PRE_APERTURA_COMB;
                Serial.println("  -> Substate: Pre_apertura_comb");
            }
            break;

        case STATE_FASE_BALISTICA:
            if (isEntry) { Serial.println("[STATE] Fase Balística"); isEntry = false; }
            // Monitoring physics/telemetry
            break;

        case STATE_FRENANDO:
            if (isEntry) { Serial.println("[STATE] Frenando"); isEntry = false; }
            break;

        case STATE_APOGEO:
            if (isEntry) { Serial.println("[STATE] Apogeo Reached"); isEntry = false; }
            break;

        case STATE_APERTURA:
            if (isEntry) { Serial.println("[STATE] Apertura"); isEntry = false; }
            break;

        case STATE_DESCENSO_RAPIDO:
            if (isEntry) { Serial.println("[STATE] Descenso Rápido"); isEntry = false; }
            break;

        case STATE_DESCENSO_LENTO:
            if (isEntry) { Serial.println("[STATE] Descenso Lento"); isEntry = false; }
            break;

        case STATE_ATERRIZAJE:
            if (isEntry) { Serial.println("[STATE] ¡Aterrizaje Exitoso!"); isEntry = false; }
            break;
    }
}

// --- Transition / Guard Conditions Logic ---
void handleStateTransitions() {
    switch (currentState) {

        case STATE_BUSCANDO_CONEXION:
            // Simulating events via Serial inputs or hardware flags
            if (g_cmd == "EV_CONECTADO") {
                currentState = STATE_ESPERA_INICIO;
                isEntry = true;
                g_cmd = "";
            } else if (g_cmd == "EV_FALLIDO") {
                Serial.println("Re-evaluating Connection...");
                currentState = STATE_BUSCANDO_CONEXION;
                isEntry = true;
                g_cmd = "";
            }
            break;

        case STATE_ESPERA_INICIO:
            if (g_cmd == "START") {
                g_vel_transmision = RAPIDA;
                init_rutina_propulsion();
                currentState = STATE_PROPULSION;
                isEntry = true;
                g_cmd = "";
            }
            break;

        case STATE_PROPULSION:
            if (completar_flag) {
                apagar_motor();
                completar_flag = false;
                currentState = STATE_FASE_BALISTICA;
                isEntry = true;
            }
            break;

        case STATE_FASE_BALISTICA:
            if (completar_flag) {
                init_rutina_frenado_aerodinamico();
                completar_flag = false;
                currentState = STATE_FRENANDO;
                isEntry = true;
            }
            break;

        case STATE_FRENANDO:
            if (vel_vertical <= 0.0) { // Apogee checkpoint
                // Action: [Completar o eliminar]
                currentState = STATE_APOGEO;
                isEntry = true;
            }
            break;

        case STATE_APOGEO:
            if (completar_flag) {
                desplegar_droge();
                completar_flag = false;
                currentState = STATE_APERTURA;
                isEntry = true;
            }
            break;

        case STATE_APERTURA:
            if (droge_estabilizado()) {
                // Action: [Completar]
                currentState = STATE_DESCENSO_RAPIDO;
                isEntry = true;
            }
            break;

        case STATE_DESCENSO_RAPIDO:
            if (altura <= 250.0 || vel_vertical <= 50.0) {
                desplegar_paracaidas_principal();
                currentState = STATE_DESCENSO_LENTO;
                isEntry = true;
            }
            break;

        case STATE_DESCENSO_LENTO:
            if (vel_vertical <= 5.0 || altura <= 50.0) {
                g_vel_transmision = LENTA;
                currentState = STATE_ATERRIZAJE;
                isEntry = true;
            }
            break;

        case STATE_ATERRIZAJE:
            // Terminal state
            break;
    }
}