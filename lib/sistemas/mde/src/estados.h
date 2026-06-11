//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_ESTADOS_H
#define ACEMA_CTLR_ESTADOS_H


/**
 * @enum Estado
 * @brief Representa los diferentes estados de una máquina de estados para un proceso.
 *
 * Este enum define los distintos estados que el sistema puede atravesar durante
 * la ejecución de su flujo de operaciones.
 *
 * Estados:
 * - ST_BUSCANDO_CONEXION: El sistema está intentando establecer una conexión.
 * - ST_ESPERA_INICIO: El sistema está esperando el inicio del proceso.
 * - ST_PROPULSION: El sistema se encuentra en la fase de propulsión activa.
 * - ST_FASE_BALISTICA: El sistema está en la fase balística (movimiento sin propulsión).
 * - ST_FRENANDO: El sistema está en la fase de desaceleración.
 * - ST_APOGEO: El sistema ha alcanzado el punto más alto de su trayectoria.
 * - ST_APERTURA: El sistema está en el proceso de despliegue (por ejemplo, de paracaídas).
 * - ST_DESCENSO_RAPIDO: El sistema está en una fase de descenso rápido.
 * - ST_DESCENSO_LENTO: El sistema está en una fase de descenso controlado y lento.
 * - ST_ATERRIZAJE: El sistema ha alcanzado el suelo o completado su descenso.
 */
typedef enum FlightState { // TODO: Chequear que los Estados estén completos y sean los indicados.
    ST_BUSCANDO_CONEXION, // fase de inicio
    ST_ESPERA_INICIO, // 
    ST_PROPULSION,
    ST_FASE_BALISTICA,
    ST_FRENANDO,
    ST_APOGEO,  // se abre drogue
    ST_DESCENSO_RAPIDO,
    ST_DESCENSO_LENTO,
    ST_ATERRIZAJE
} Estado;

extern Estado curr_state; // Estado actual de la máquina de estados


#endif //ACEMA_CTLR_ESTADOS_H
