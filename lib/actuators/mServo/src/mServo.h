//
// Created by zhl on 6/4/26.
//

#ifndef ACEMA_CTLR_SERVO_H
#define ACEMA_CTLR_SERVO_H


#include <Arduino.h>

#include "ESP32Servo.h"

// Configuración del Servo (hardware y comportamiento)
typedef struct {
    uint8_t pinServo;     // Pin PWM del servo
    uint8_t pinPot;       // Pin ADC del potenciómetro (opcional)
    float   pendiente;    // Grados por unidad ADC (ej. 180 / 4095)
    float   alpha;          // EMA para filtrado de lectura
    float   deadband;       // Margen de error en grados
    uint16_t minPulse;      // Pulso mínimo en µs para 0°
    uint16_t maxPulse;      // Pulso máximo en µs para 180°
    uint8_t id;             // Identificador opcional
} mServoConfig_t;

class mServo {
public:
    // Constructor que encapsula la inicialización
    explicit mServo(mServoConfig_t* cfg);

    // API de control (sin inicialización externa)
    float leerAngulo();
    void setAngulo(float ang, uint16_t degPerSec);
    void step(bool autoActualizacion);
    bool enAnguloTarget() const;
    bool enAngulo(float pos) const;

private:
    mServoConfig_t* cfg;
    Servo servo;
    float angActual;
    float angTarget;
    uint32_t intervaloPaso;
    uint32_t ultimoPasoMicros;
    bool estaMoviendo;
    bool tieneError;
    uint8_t ID;

    // Helper para lectura bruta (sin EMA)
    float leerAnguloBruto() const;
};


#endif //ACEMA_CTLR_SERVO_H
