#include "sensores.h"
#include "SerialPrint.h"

/**
 * @brief Inicializa y configura todos los sensores del sistema
 * @return 0 si la configuración fue exitosa, != 0 si hubo error
 */
int sensores_configurar(void) {
    SerialPrint::msg("[SENSORES] Configurando sensores...");

    // TODO: Implementar la configuración real de los sensores
    // - Inicializar MPU6050
    // - Inicializar BMP280
    // - Inicializar GPS

    return 0;
}

/**
 * @brief Lee los datos de todos los sensores y actualiza la estructura
 * @param data_raw Puntero a la estructura donde se almacenarán los datos crudos
 * @return 0 si la lectura fue exitosa, != 0 si hubo error
 */
int sensores_leer(data_raw_t* data_raw) {
    if (data_raw == nullptr) {
        return -1;
    }

    // TODO: Implementar la lectura real de los sensores
    // - Leer datos del MPU6050
    // - Leer datos del BMP280
    // - Leer datos del GPS
    // - Procesar y filtrar los datos

    // Valores por defecto (stub)
    // data_raw->altura = 0.0f;
    // data_raw->inclinacion = 0.0f;
    // data_raw->aceleracion = 0.0f;
    // data_raw->vel_vertical = 0.0f;
    // data_raw->variacion_aceleracion = 0.0f;
    // data_raw->estado_hardware = HARDWARE_OK;

    return 0;
}

/**
 * @brief Obtiene el estado actual del hardware de los sensores
 * @return Estado del hardware (HARDWARE_OK, HARDWARE_WARNING, o HARDWARE_FALLA_CRITICA)
 */
uint8_t sensores_obtener_estado_hardware(void) {
    // TODO: Implementar la verificación real del estado del hardware
    return HARDWARE_OK;
}

