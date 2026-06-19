#ifndef ACEMA_CTLR_SENSORES_H
#define ACEMA_CTLR_SENSORES_H

#include <cstdint>

#include "data.h"

#ifdef __cplusplus
extern "C" {
#endif


// Estado del hardware
#define HARDWARE_OK                0x00
#define HARDWARE_WARNING           0x01
#define HARDWARE_FALLA_CRITICA     0x02

/**
 * @brief Inicializa y configura todos los sensores del sistema
 * @return 0 si la configuración fue exitosa, != 0 si hubo error
 */
int sensores_configurar(void);

/**
 * @brief Lee los datos de todos los sensores y actualiza la estructura
 * @param data_raw Puntero a la estructura donde se almacenarán los datos crudos
 * @return 0 si la lectura fue exitosa, != 0 si hubo error
 */
int sensores_leer(data_raw_t* data_raw);

/**
 * @brief Obtiene el estado actual del hardware de los sensores
 * @return Estado del hardware (HARDWARE_OK, HARDWARE_WARNING, o HARDWARE_FALLA_CRITICA)
 */
uint8_t sensores_obtener_estado_hardware(void);

#ifdef __cplusplus
}
#endif

#endif // ACEMA_CTLR_SENSORES_H

