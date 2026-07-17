//
// Created by zhl on 6/4/26.
//

#ifndef ACEMA_CTLR_SERVO_H
#define ACEMA_CTLR_SERVO_H


#include <Arduino.h>

#include "ESP32Servo.h"

// Configuración del Servo (sólo hardware y límites de pulso)
typedef struct {
    uint8_t  pinServo;      // Pin PWM conectado al servo
    uint16_t minPulse;      // Pulso mínimo en µs para 0°
    uint16_t maxPulse;      // Pulso máximo en µs para 180°
    uint8_t  id;            // Identificador opcional del servo
} mServoConfig_t;

class mServo {
public:
    // Constructor que encapsula la configuración
    explicit mServo(mServoConfig_t* cfg);

    // Inicializa el hardware y posiciona el servo en 0°
    bool init();

    // Comanda el servo directamente al ángulo especificado a máxima velocidad (0.0° - 180.0°)
    void sendAngulo(float ang);

    // Retorna el último ángulo comandado
    float getLastSentAngulo() const;

    // Retorna el ID asignado
    uint8_t getID() const;

private:
    mServoConfig_t* _cfg;
    Servo _servo;
    float angActual;
    uint8_t ID;
};


#endif //ACEMA_CTLR_SERVO_H
