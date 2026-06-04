//
// Created by zhl on 5/19/26.
//

#include "maquina_de_estados.h"


// Initialize Global Variables
FlightState currentState = STATE_BUSCANDO_CONEXION;
PropulsionSubState propSubState = SUB_PRE_APERTURA_OX;
TransmissionSpeed g_vel_transmision = LENTA;
String g_cmd = "";
float vel_vertical = 12.0;
float altura = 600.0;
bool completar_flag = false;
bool isEntry = true; // Tracks entry actions






