//
// Created by lucaz on 11/7/2026.
//

#include "mFlash.h"

mFlash::mFlash(uint8_t csPin) : _flash(csPin), _currentLogAddr(ADDR_LOG) {}

bool mFlash::begin(size_t lenDatos) {
    Serial.println("[mFlash] ENTRANDO A BEGIN");
    if (!_flash.begin()) {
        Serial.println("[mFlash] ERROR: _flash.begin() de SPIMemory devolvio false.");
        return false;
    }

    Serial.println("[mFlash] Escaneando memoria. Etapa 1 (Saltos de 4KB)...");

    // --- ETAPA 1: BÚSQUEDA GRUESA (Saltos de 4KB) ---
    uint32_t sectorAddr = ADDR_LOG; // Sector 1[cite: 1]
    uint32_t escapeCounter = 0;     // Evita bucles infinitos en hardware roto

    while (sectorAddr < FLASH_LIMIT) { // Límite de 16 MB[cite: 1]
        uint32_t token = 0;
        
        // Leemos los primeros 4 bytes del sector[cite: 2]
        _flash.readByteArray(sectorAddr, (uint8_t*)&token, sizeof(token));
        
        // DEBUG SENSITIVO: Hacemos un print cada 10 sectores para ver si avanza o lee basura
        if (escapeCounter % 10 == 0) {
            Serial.printf("[mFlash] Addr: 0x%06X -> Token leido: 0x%08X\n", sectorAddr, token);
        }

        // Si es 0xFFFFFFFF significa que el sector está completamente limpio[cite: 2]
        if (token == 0xFFFFFFFF) {
            Serial.printf("[mFlash] Sector limpio encontrado en Addr: 0x%06X\n", sectorAddr);
            break; 
        }
        
        sectorAddr += 4096; // Saltamos al siguiente sector[cite: 2]
        escapeCounter++;

        // Salvavidas: si recorre más de 4096 sectores (16MB totales), salimos sí o sí
        if (escapeCounter > 4096) {
            Serial.println("[mFlash] ERROR: Superado el limite de sectores sin encontrar 0xFFFFFFFF. Memoria corrupta o ausente.");
            return false;
        }
    }

    Serial.println("[mFlash] Pasando a Etapa 2...");
    // --- ETAPA 2: BÚSQUEDA FINA (Registro por registro) ---
    uint32_t fineAddr = (sectorAddr > ADDR_LOG) ? (sectorAddr - 4096) : ADDR_LOG; //[cite: 2]
    escapeCounter = 0;

    while (fineAddr < sectorAddr && fineAddr < FLASH_LIMIT) { //[cite: 2]
        uint32_t token = 0;
        _flash.readByteArray(fineAddr, (uint8_t*)&token, sizeof(token));
        
        if (token == 0xFFFFFFFF) { //[cite: 2]
            break; 
        }
        fineAddr += lenDatos; //[cite: 2]
        escapeCounter++;

        if (escapeCounter > 1000) { // Límite de seguridad fina
            Serial.println("[mFlash] ERROR: Bucle infinito en Etapa 2.");
            return false;
        }
    }

    _currentLogAddr = fineAddr; //[cite: 2]

    if (_currentLogAddr > ADDR_LOG) { //[cite: 2]
        uint32_t sectorActual = _currentLogAddr / 4096; //[cite: 2]
        _currentLogAddr = (sectorActual + 1) * 4096; //[cite: 2]
    }
    
    Serial.printf("[mFlash] Inicializacion finalizada. _currentLogAddr: 0x%06X\n", _currentLogAddr);
    return true;
}
/*
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
*/
// --- SECCIÓN CONFIGURACIÓN (PREVUELO) ---
void mFlash::guardarConfig(const void* config, size_t lenConfig) {
    _flash.eraseSector(ADDR_CONFIG); 
    
    // Castea el void* a uint8_t* para la librería
    _flash.writeByteArray(ADDR_CONFIG, (uint8_t*)config, lenConfig);
}

bool mFlash::recuperarConfig(void* config, size_t lenConfig) {
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