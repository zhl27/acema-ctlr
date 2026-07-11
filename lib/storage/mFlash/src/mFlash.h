//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_MFLASH_H
#define ACEMA_CTLR_MFLASH_H
#include <stdbool.h>
#include <stdint.h>


class mFlash {
public:
    // Constructor: Por defecto usa el GPIO 4 para el pin CS_FLASH
    explicit mFlash(uint8_t csPin = 4) : _csPin(csPin) {}

    // Inicializa los pines y el bus SPI estándar del ESP32
    void init() const;

    // Ejecuta la prueba de lectura e imprime el diagnóstico en el puerto Serial
    // bool testConnection(Stream &serialPort = Serial) const;
    bool testConnection(Stream &serialPort);

private:
    uint8_t _csPin = 0;

    // Comandos y configuraciones internas (Ocultas al usuario)
    const uint8_t _CMD_READ_JEDEC_ID = 0x9F;
    const uint32_t _SPI_SPEED = 10000000; // 10 MHz por seguridad

    // Método privado de bajo nivel para interactuar con el bus SPI
    void _leerChipID(uint8_t &manufacturerID, uint8_t &memoryTypeID, uint8_t &capacityID);
};


#endif //ACEMA_CTLR_MFLASH_H
