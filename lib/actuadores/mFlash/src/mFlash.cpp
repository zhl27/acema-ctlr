//
// Created by zhl on 6/6/26.
//

#include "mFlash.h"

// TODO: Falta revisión general de mFlash

mFlash::mFlash(uint8_t csPin) {
    _csPin = csPin;
}

void mFlash::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    // Inicializa la línea SPI por defecto del ESP-WROOM-32 (VSPI) // TODO: Revisar, no queremos Virtual SPI
    SPI.begin();
}

void mFlash::_leerChipID(uint8_t &manufacturerID, uint8_t &memoryTypeID, uint8_t &capacityID) {
    SPI.beginTransaction(SPISettings(_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_csPin, LOW);
    SPI.transfer(_CMD_READ_JEDEC_ID);

    manufacturerID = SPI.transfer(0x00);
    memoryTypeID   = SPI.transfer(0x00);
    capacityID     = SPI.transfer(0x00);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

bool mFlash::testConnection(Stream &serialPort) {
    uint8_t manufID = 0, memTypeID = 0, capID = 0;

    // Llamada interna al método privado
    _leerChipID(manufID, memTypeID, capID);

    serialPort.println(F("\n--- Test de Memoria Flash (Clase mFlash) ---"));
    serialPort.print(F(" -> ID de Fabricante (Winbond 0xEF): 0x"));
    serialPort.println(manufID, HEX);
    serialPort.print(F(" -> Tipo de Memoria (Suele ser 0x40): 0x"));
    serialPort.println(memTypeID, HEX);
    serialPort.print(F(" -> ID de Capacidad (128Mb es 0x18): 0x"));
    serialPort.println(capID, HEX);

    if (manufID == 0xEF) {
        serialPort.println(F("\n[ÉXITO] ¡Conexión mFlash establecida correctamente!"));
        return true;
    } else if (manufID == 0x00 || manufID == 0xFF) {
        serialPort.println(F("\n[ERROR] mFlash no detectada. Verifica SCK(18), MISO(19) y MOSI(23)."));
        return false;
    } else {
        serialPort.println(F("\n[ADVERTENCIA] Respuesta recibida, pero el ID no coincide con Winbond."));
        return false;
    }
}