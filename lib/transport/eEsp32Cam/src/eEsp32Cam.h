//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_EESP32CAM_H
#define ACEMA_CTLR_EESP32CAM_H
#include <HardwareSerial.h>


class eEsp32Cam {
private:
    HardwareSerial* _uart;
    int _rxPin;
    int _txPin;
    unsigned long _baudrate;
    uint32_t _timeoutMs = 100; // Timeout crítico de 100ms para no bloquear el vuelo

    // Función interna para calcular un checksum XOR básico y validar integridad
    static uint8_t _calcularChecksum(const String& comando);

public:
    explicit eEsp32Cam(HardwareSerial& uartPort = Serial2, const int rxPin = 16, const int txPin = 17, unsigned long baudrate = 921600) : _uart(&uartPort), _rxPin(rxPin), _txPin(txPin), _baudrate(baudrate) {
    }

    void init() const {
        _uart->begin(_baudrate, SERIAL_8N1, _rxPin, _txPin);
    }

    // Envía un comando formateado con Checksum para garantizar integridad
    bool enviar_comando(const String& cmdType, const String& payload = "") const;

    // Espera una respuesta de la ESP-CAM de forma no bloqueante (con timeout)
    String recibir_respuesta() const;

    // Envía un comando y espera validación ($OK o $DATA)
    bool ejecutar_comando_sincrono(const String& cmdType, const String& payload, String& respuestaOut) const;
};

#endif //ACEMA_CTLR_EESP32CAM_H
