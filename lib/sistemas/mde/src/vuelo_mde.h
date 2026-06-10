//
// Created by zhl on 6/8/26.
//

#ifndef ACEMA_CTLR_MDE_H
#define ACEMA_CTLR_MDE_H


/**
 * @file vuelo_mde.h
 * @brief Interfaz de la Máquina de Estados para la secuencia de vuelo del cohete.
 */

#ifndef VUELO_MDE_H
#define VUELO_MDE_H

#include <stdint.h>
#include <stdbool.h>

// --- Enumeraciones ---

/** @brief Definición de los estados secuenciales del vuelo */
typedef enum {
    ST_BUSCANDO_CONEXION = 0,
    ST_ESPERA_INICIO,
    ST_PROPULSION,
    ST_FASE_BALISTICA,
    ST_FRENANDO,
    ST_APOGEO,
    ST_APERTURA,
    ST_DESCENSO_RAPIDO,
    ST_DESCENSO_LENTO,
    ST_ATERRIZAJE,
    ST_VUELO_ERROR,
    ST_VUELO_MAX // Límite de seguridad para la tabla de estados
  } estadoVuelo_t;

/** @brief Códigos de error críticos del sistema */
typedef enum {
    ERR_NINGUNO = 0,
    ERR_CONEXION_GSE,
    ERR_CONDICION_INICIAL,
    ERR_TRAYECTORIA_PELIGROSA,
    ERR_DROGUE_FALLIDO
  } codigoError_t;

// --- Estructuras de Datos ---

/** * @brief Contexto físico y de estado del cohete.
 */
typedef struct {
    estadoVuelo_t estado;
} cohete_t;

/** * @brief Parámetros y variables de control específicos del ensayo/vuelo.
 */
typedef struct {
    uint8_t intentosLora;
    float umbralVelFrenado; // Velocidad límite para iniciar el frenado
} datosVuelo_t;

// --- Prototipos de Funciones ---

/** * @brief Firma estándar para las funciones de estado de la MdE.
 */
typedef void (*vuelo_estado_func_t)(cohete_t* self, datosVuelo_t* datos);

/**
 * @brief Función principal de actualización de la Máquina de Estados.
 * @details Debe ser llamada continuamente dentro del loop principal (scheduler).
 * * @param self Puntero a la instancia del cohete.
 * @param datos Puntero a los datos/parámetros del vuelo actual.
 */
void vuelo_mde_actualizar(cohete_t* self, datosVuelo_t* datos);

#endif // VUELO_MDE_H



#endif //ACEMA_CTLR_MDE_H
