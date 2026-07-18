//
// Created by lucaz on 11/7/2026.
//

#include "mFlash.h"

mFlash::mFlash(uint8_t csPin) : _flash(csPin), _currentLogAddr(ADDR_LOG) {}


// Al arrancar, le pasamos el tamaño del struct de log para que calcule los offsets
bool mFlash::begin(size_t lenDatos) {
    if (!_flash.begin()) return false;

    // --- ETAPA 1: BÚSQUEDA GRUESA (Saltos de 4KB) ---
    uint32_t sectorAddr = ADDR_LOG;
    while (sectorAddr < FLASH_LIMIT) {
        uint32_t token;
        // Leemos los primeros 4 bytes del sector (habitualmente el timestamp)
        _flash.readByteArray(sectorAddr, (uint8_t*)&token, sizeof(token));
        
        // Si es 0xFFFFFFFF significa que el sector está completamente limpio
        if (token == 0xFFFFFFFF) {
            break; 
        }
        sectorAddr += 4096; // Saltamos al siguiente sector
    }

    // --- ETAPA 2: BÚSQUEDA FINA (Registro por registro) ---
    // Retrocedemos un sector completo (donde sabemos que hay datos) para hilar fino
    uint32_t fineAddr = (sectorAddr > ADDR_LOG) ? (sectorAddr - 4096) : ADDR_LOG;
    
    while (fineAddr < sectorAddr && fineAddr < FLASH_LIMIT) {
        uint32_t token;
        _flash.readByteArray(fineAddr, (uint8_t*)&token, sizeof(token));
        
        if (token == 0xFFFFFFFF) {
            break; // Encontramos el primer espacio vacío exacto
        }
        fineAddr += lenDatos; // Avanzamos el tamaño exacto de la estructura
    }

    _currentLogAddr = fineAddr; 

    // SI DETECTA QUE NO ESTÁ AL INICIO DEL LOG (Hubo un reinicio y ya hay datos)
    if (_currentLogAddr > ADDR_LOG) {
        // Redondea la dirección al inicio del SIGUIENTE sector de 4KB
        // Ej: si quedó en 0x1234, salta a 0x2000
        uint32_t sectorActual = _currentLogAddr / 4096;
        _currentLogAddr = (sectorActual + 1) * 4096;
    }
    return true;
}

// --- SECCIÓN CONFIGURACIÓN (PREVUELO) ---
void mFlash::guardarConfig(const void* config, size_t lenConfig) {
    _flash.eraseSector(ADDR_CONFIG); 
    
    // Castea el void* a uint8_t* para la librería
    _flash.writeByteArray(ADDR_CONFIG, (uint8_t*)config, lenConfig);
}

bool mFlash::cargarConfig(void* config, size_t lenConfig) {
    return _flash.readByteArray(ADDR_CONFIG, (uint8_t*)config, lenConfig);
}

// --- SECCIÓN TELEMETRÍA / FILTRADO ---
void mFlash::resetearLog() {
    if (_currentLogAddr <= ADDR_LOG) return; // Ya está limpio, no hacemos nada

    uint32_t startSector = ADDR_LOG / 4096;
    // Redondeamos hacia arriba para asegurarnos de borrar el último sector usado a medias
    uint32_t endSector = (_currentLogAddr + 4095) / 4096; 

    for (uint32_t i = startSector; i < endSector; i++) {
        _flash.eraseSector(i * 4096);
    }

    _currentLogAddr = ADDR_LOG; // Reset al origen
}

bool mFlash::guardarPuntoLog(const void* datos, size_t lenDatos) {
    // Verificación de fin de memoria física
    if (_currentLogAddr + lenDatos >= FLASH_LIMIT) return false; 
    
    if (_flash.writeByteArray(_currentLogAddr, (uint8_t*)datos, lenDatos)) {
        _currentLogAddr += lenDatos; // El puntero avanza según el tamaño dinámico enviado
        return true;
    }
    return false;
}

// --- MÉTODOS DE LECTURA POST-VUELO ---

// Al ser genérico, el cálculo depende del tamaño del struct
uint32_t mFlash::getCantidadRegistros(size_t lenDatos) {
    return (_currentLogAddr - ADDR_LOG) / lenDatos;
}

// Lee un registro calculando el offset según el tamaño indexado
bool mFlash::leerPuntoLog(uint32_t index, void* datos, size_t lenDatos) {
    uint32_t targetAddr = ADDR_LOG + (index * lenDatos);
    
    // Evitamos que intente leer más allá de lo que se ha escrito en este encendido
    if (targetAddr + lenDatos > _currentLogAddr) return false; 
    
    return _flash.readByteArray(targetAddr, (uint8_t*)datos, lenDatos);
}