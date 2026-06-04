//
// Created by zhl on 5/19/26.
//

#ifndef ACEMA_CTLR_MAQUINADEESTADOS_H
#define ACEMA_CTLR_MAQUINADEESTADOS_H



// --- Telemetry / Mock Global Variables ---
extern FlightState currentState;
extern String g_cmd;
extern float vel_vertical;
extern float altura;
extern bool completar_flag;

enum TransmissionSpeed { LENTA, RAPIDA };
extern TransmissionSpeed g_vel_transmision;

// --- State Machine Functions ---
void runStateMachine();
void handleStateTransitions();




void * (mde_cohete*)(void*) [4];
typedef int (*mde_cohete)(int, int);
mde_cohete lista_mde_cohete[4] = {do_state_s1, do_state_s2, do_state_s3, do_state_s4};

// Acciones
void do_state_s1();
void do_state_s2();
void do_state_s3();
void do_state_s4();
void do_state_s5();
void do_state_s6();
void do_state_s7();





#endif //ACEMA_CTLR_MAQUINADEESTADOS_H
