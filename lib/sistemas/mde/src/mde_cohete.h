//
// Created by zhl on 6/11/26.
//

#ifndef ACEMA_CTLR_MDE_COHETE_H
#define ACEMA_CTLR_MDE_COHETE_H


#include "data.h"


/** @enum estado_vuelo_t
 * @brief Definición de los estados secuenciales del vuelo
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
typedef enum {
    ST_INIT = 0, // Estado de inicialización
    ST_BUSCANDO_CONEXION,
    ST_ESPERA_INICIO,
    ST_PROPULSION,
    ST_FASE_BALISTICA,
    ST_FRENANDO,
    ST_APOGEO, // se abre drogue
    ST_DESCENSO_RAPIDO,
    ST_DESCENSO_LENTO,
    ST_ATERRIZAJE,
    ST_ERROR,
    ST_NULL // Límite de seguridad para la tabla de estados
  } estado_vuelo_t;

/** @enum cod_error_t
 * @brief Códigos de error críticos del sistema */
// TODO: Completar con los códigos de error que se pueden presentar en el INSTANTE
typedef enum {
    ERR_NINGUNO = 0,
    ERR_CONEXION_GSE,
    ERR_CONDICION_INICIAL,
    ERR_TRAYECTORIA_PELIGROSA,
    ERR_DROGUE_FALLIDO
  } cod_error_t;


typedef struct {
    estado_vuelo_t estado_actual; ///< Estado actual de la máquina de estados.
    cod_error_t error; ///< Código de error actual

    data_all_t contexto_fisico; ///< No confundir con `data_all_t* datos_sensores`.
    // TODO: completar con datos globales que tengan relacion con el cohete
} mde_data_t;

// DECLARAMOS LA VARIABLE GLOBAL DEL SISTEMA GLOBAL
// Y DEFINIMOS VALORES INICIALES
mde_data_t COHETE = {
    .estado_actual = ST_INIT,
    .error = ERR_NINGUNO,
    .contexto_fisico = {
        .data_raw = {
            .bmp = {0},
            .mpc = {0},
            .gps = {0},
            .timestamp = 0
        },
        .posicion_relativa = 0,
        .velocidad = 0,
        .momentum = 0,
    },
};

void mde_cohete_actualizar(data_all_t* datos_sensores);


#endif //ACEMA_CTLR_MDE_COHETE_H
