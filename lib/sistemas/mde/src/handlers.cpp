//
// Created by zhl on 6/3/26.
//

#include "handlers.h"
#include "acciones.h"
#include "estados.h"
#include "SerialPrint.h"

static bool isEntry = true;

// void handle_mde(){
//     if( estado >=  N_EStado) {
//         estado =flash.leerEstado ();
//         if(){
//             handle_error();
//         }
//     }
//     mde_cohete[estado]();
// }

// void handle_error() {
//     apagar_motor();
//     lanzar_paracaida();
//     estado = descenso;
// }


// --- State Machine Core Execution ---
void runStateMachine() {
    switch (curr_state) {

        case ST_BUSCANDO_CONEXION:
            if (isEntry) {
                SerialPrint::msg("[STATE] Buscando Conexión");
                f_st_configurar_sensores();
                f_st_configurar_actuadores();
                isEntry = false;
            }
            f_st_conectar_GSE();
            break;

        case ST_ESPERA_INICIO:
            if (isEntry) {
                SerialPrint::msg("[STATE] Espera Inicio");
                // g_vel_transmision = LENTA;
                isEntry = false;
            }
            f_st_get_cmd_GSE();
            break;

        case ST_PROPULSION:
            if (isEntry) {
                SerialPrint::msg("[STATE] Propulsion");
                f_st_init_rutina_propulsion();
                isEntry = false;
            }

            // Substate Machine Logic
            // if (propSubState == SUB_ST_PRE_APERTURA_OX) {
            //     // Perform oxygen valve tasks...
            //     propSubState = SUB_ST_PRE_APERTURA_COMB;
            //     SerialPrint::msg("  -> Substate: Pre_apertura_comb");
            // }
            break;

        case ST_FASE_BALISTICA:
            if (isEntry) { SerialPrint::msg("[STATE] Fase Balística"); isEntry = false; }
            // Monitoring physics/telemetry
            break;

        case ST_FRENANDO:
            if (isEntry) { SerialPrint::msg("[STATE] Frenando"); isEntry = false; }
            break;

        case ST_APOGEO:
            if (isEntry) { SerialPrint::msg("[STATE] Apogeo Reached"); isEntry = false; }
            break;

        case ST_APERTURA:
            if (isEntry) { SerialPrint::msg("[STATE] Apertura"); isEntry = false; }
            break;

        case ST_DESCENSO_RAPIDO:
            if (isEntry) { SerialPrint::msg("[STATE] Descenso Rápido"); isEntry = false; }
            break;

        case ST_DESCENSO_LENTO:
            if (isEntry) { SerialPrint::msg("[STATE] Descenso Lento"); isEntry = false; }
            break;

        case ST_ATERRIZAJE:
            if (isEntry) { SerialPrint::msg("[STATE] ¡Aterrizaje Exitoso!"); isEntry = false; }
            break;
    }
}

// --- Transition / Guard Conditions Logic ---
void handleStateTransitions() {
    switch (curr_state) {

        case ST_BUSCANDO_CONEXION:
            // Simulating events via Serial inputs or hardware flags
            // if (g_cmd == "EV_CONECTADO") {
            //     currentState = ST_ESPERA_INICIO;
            //     isEntry = true;
            //     g_cmd = "";
            // } else if (g_cmd == "EV_FALLIDO") {
            //     SerialPrint::msg("Re-evaluating Connection...");
            //     currentState = ST_BUSCANDO_CONEXION;
            //     isEntry = true;
            //     g_cmd = "";
            // }
            break;

        case ST_ESPERA_INICIO:
            // if (g_cmd == "START") {
            //     g_vel_transmision = RAPIDA;
            //     init_rutina_propulsion();
            //     currentState = ST_PROPULSION;
            //     isEntry = true;
            //     g_cmd = "";
            // }
            break;

        case ST_PROPULSION:
            // if (completar_flag) {
            //     apagar_motor();
            //     completar_flag = false;
            //     currentState = ST_FASE_BALISTICA;
            //     isEntry = true;
            // }
            break;

        case ST_FASE_BALISTICA:
            // if (completar_flag) {
            //     init_rutina_frenado_aerodinamico();
            //     completar_flag = false;
            //     currentState = ST_FRENANDO;
            //     isEntry = true;
            // }
            break;

        case ST_FRENANDO:
            // if (vel_vertical <= 0.0) { // Apogee checkpoint
            //     // Action: [Completar o eliminar]
            //     currentState = ST_APOGEO;
            //     isEntry = true;
            // }
            break;

        case ST_APOGEO:
            // if (completar_flag) {
            //     desplegar_droge();
            //     completar_flag = false;
            //     currentState = ST_APERTURA;
            //     isEntry = true;
            // }
            break;

        case ST_APERTURA:
            // if (droge_estabilizado()) {
            //     // Action: [Completar]
            //     currentState = ST_DESCENSO_RAPIDO;
            //     isEntry = true;
            // }
            break;

        case ST_DESCENSO_RAPIDO:
            // if (altura <= 250.0 || vel_vertical <= 50.0) {
            //     desplegar_paracaidas_principal();
            //     currentState = ST_DESCENSO_LENTO;
            //     isEntry = true;
            // }
            break;

        case ST_DESCENSO_LENTO:
            // if (vel_vertical <= 5.0 || altura <= 50.0) {
            //     g_vel_transmision = LENTA;
            //     currentState = ST_ATERRIZAJE;
            //     isEntry = true;
            // }
            break;

        case ST_ATERRIZAJE:
            // Terminal state
            break;
    }
}