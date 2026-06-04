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
 * - STATE_BUSCANDO_CONEXION: El sistema está intentando establecer una conexión.
 * - STATE_ESPERA_INICIO: El sistema está esperando el inicio del proceso.
 * - STATE_PROPULSION: El sistema se encuentra en la fase de propulsión activa.
 * - STATE_FASE_BALISTICA: El sistema está en la fase balística (movimiento sin propulsión).
 * - STATE_FRENANDO: El sistema está en la fase de desaceleración.
 * - STATE_APOGEO: El sistema ha alcanzado el punto más alto de su trayectoria.
 * - STATE_APERTURA: El sistema está en el proceso de despliegue (por ejemplo, de paracaídas).
 * - STATE_DESCENSO_RAPIDO: El sistema está en una fase de descenso rápido.
 * - STATE_DESCENSO_LENTO: El sistema está en una fase de descenso controlado y lento.
 * - STATE_ATERRIZAJE: El sistema ha alcanzado el suelo o completado su descenso.
 */
enum Estado {
    STATE_BUSCANDO_CONEXION,
    STATE_ESPERA_INICIO,
    STATE_PROPULSION,
    STATE_FASE_BALISTICA,
    STATE_FRENANDO,
    STATE_APOGEO,
    STATE_APERTURA,
    STATE_DESCENSO_RAPIDO,
    STATE_DESCENSO_LENTO,
    STATE_ATERRIZAJE
};
Estado curr_state = STATE_S1; // estado inicial


#endif //ACEMA_CTLR_ESTADOS_H
