//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_EESP32CAM_H
#define ACEMA_CTLR_EESP32CAM_H
#include <HardwareSerial.h>
#include <stdbool.h>
#include <stdint.h>


#define CMD_CAM_CTRL      0x10  // Param: 1 = Start, 0 = Stop
#define CMD_SD_DUMP_START 0x20  // Param: 0
#define ACK_SUCCESS       0xFF  // ESP-B replies with this
#define NACK_ERROR        0xEE  // ESP-B replies if SD card fails


class eEsp32Cam {
private:
    HardwareSerial* _uart;
    int _rxPin;
    int _txPin;
    unsigned long _baudrate;
    uint32_t _timeoutMs = 100; // Timeout crítico de 100ms para no bloquear el vuelo

    // Función interna para calcular un checksum XOR básico y validar integridad
    // static uint8_t _calcularChecksum(const char* comando);

    static uint8_t _calculate_crc8(uint8_t opcode, uint8_t param);

public:
    explicit eEsp32Cam(HardwareSerial& uartPort = Serial2, const int rxPin = 16, const int txPin = 17, unsigned long baudrate = 921600) : _uart(&uartPort), _rxPin(rxPin), _txPin(txPin), _baudrate(baudrate) {
    }

    void init() const {
        _uart->begin(_baudrate, SERIAL_8N1, _rxPin, _txPin);
    }

    // Envía un comando formateado con Checksum para garantizar integridad
    bool sendCommandToCam(uint8_t opcode, uint8_t param);
};

#endif //ACEMA_CTLR_EESP32CAM_H
