//
// Created by lucaz on 11/7/2026.
// Modificado: JoJoe 18/7/2026
//

#ifndef ACEMA_CTLR_MFLASH_H
#define ACEMA_CTLR_MFLASH_H
#include <SPIMemory.h>

class mFlash {
private:
    SPIFlash _flash;
    const uint32_t ADDR_CONFIG = 0x000000; // Sector 0
    const uint32_t ADDR_LOG    = 0x001000; // Sector 1
    const uint32_t FLASH_LIMIT = 0x1000000; // 16 MB (Límite físico de la W25Q128)
    uint32_t _currentLogAddr;

public:
    explicit mFlash(uint8_t csPin);
    
    // Al arrancar, le pasamos el tamaño del struct de log para que calcule los offsets
    bool begin(size_t lenDatos);

    // --- SECCIÓN CONFIGURACIÓN (PREVUELO) ---
    void guardarConfig(const void* config, size_t lenConfig);

    bool recuperarConfig(void* config, size_t lenConfig);

    // --- SECCIÓN TELEMETRÍA / FILTRADO ---
    void resetearLog();

    bool guardarPuntoLog(const void* datos, size_t lenDatos);

    // --- MÉTODOS DE LECTURA POST-VUELO ---

    // Al ser genérico, el cálculo depende del tamaño del struct 
    uint32_t getCantidadRegistros(size_t lenDatos);

    // Lee un registro calculando el offset según el tamaño indexado
    bool leerPuntoLog(uint32_t index, void* datos, size_t lenDatos);
};


#endif //ACEMA_CTLR_MFLASH_H
