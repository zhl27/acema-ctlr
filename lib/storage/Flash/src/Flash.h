//
// Created by zhl on 6/24/26.
//

#ifndef ACEMA_CTLR_FLASH_H
#define ACEMA_CTLR_FLASH_H

#include <Arduino.h>
#include <SPI.h>

class Flash {
public:
    // Constructor: Por defecto usa el GPIO 4 para el pin CS_FLASH
    explicit Flash(uint8_t csPin = 4);

    // Inicializa los pines y el bus SPI estándar del ESP32
    void begin() const;

    // Ejecuta la prueba de lectura e imprime el diagnóstico en el puerto Serial
    bool testConnection(Stream &serialPort = Serial) const;

private:
    uint8_t _csPin = 0;

    // Comandos y configuraciones internas (Ocultas al usuario)
    const uint8_t _CMD_READ_JEDEC_ID = 0x9F;
    const uint32_t _SPI_SPEED = 10000000; // 10 MHz por seguridad

    // Método privado de bajo nivel para interactuar con el bus SPI
    void _leerChipID(uint8_t &manufacturerID, uint8_t &memoryTypeID, uint8_t &capacityID) const;
};


#endif //ACEMA_CTLR_FLASH_H
