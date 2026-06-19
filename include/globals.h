//
// Created by zhl on 6/2/26.
//

#ifndef ACEMA_CTLR_GLOBALS_H
#define ACEMA_CTLR_GLOBALS_H

#include "data.h"

#include "config.h"

/*************************/
/*         GPIOs         */
/*************************/

#define BUZZER_PIN 25
#define WIRE_SDA 21
#define WIRE_SCL 22


/*************************/
/*    DIRECCIONES I2C    */
/*************************/

#define MPU_ADDR 0x69
#define BMP280_ADDR 0x77


/*************************/
/*         MDE         */
/*************************/

/** @enum estadoVuelo_t
 * @brief Definición de los estados secuenciales del vuelo */
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
    ST_NONE // Límite de seguridad para la tabla de estados
  } estadoVuelo_t;

/** @ENUM codigoError_t
 * @brief Códigos de error críticos del sistema */
typedef enum {
    ERR_NINGUNO = 0,
    ERR_CONEXION_GSE,
    ERR_CONDICION_INICIAL,
    ERR_TRAYECTORIA_PELIGROSA,
    ERR_DROGUE_FALLIDO
  } codigoError_t;



#endif //ACEMA_CTLR_GLOBALS_H
