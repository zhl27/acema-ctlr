//
// Created by zhl on 6/25/26.
//

#include "Actuators.h"


// ====================================================================
// Inicialización de los miembros estáticos de la clase Actuators
// ====================================================================

// Inicializamos el buzzer usando el pin por defecto (25) o puedes especificar otro.
mBuzzer Actuators::_buzzer(25);

// Inicializamos el servo usando configuración específica, debemos pasarle una estructura mServoConfig_t
mServoConfig_t config_airbrake = {
    .pinServo  = 27,        // Pin PWM del servo
    // .pinPot    = 34,        // Cualquier pin ADC (no importa si flota, se ignorará)
    // .pendiente = 0.0f,      // 0 grados por unidad ADC
    // .alpha     = 0.0f,      // <--- CLAVE: 0% de peso a la lectura analógica
    // .deadband  = 0.5f,      // Pequeño margen para evitar rebotes en los cálculos
    .minPulse  = 500,       // Pulso en µs para 0°
    .maxPulse  = 2500,      // Pulso en µs para 180°
    .id        = 1
};
mServo Actuators::_servo(&config_airbrake);

// Inicializamos los pirotécnicos basándonos en los pines sugeridos en mPyro.h
// Drogue: Activar en D12, Continuidad en D35, Umbral 1000
mPyro Actuators::_pyro_pcaidas_drogue(12, 35, 1000);

// Principal (Ppal): Activar en D13, Continuidad en VN (39), Umbral 1000
mPyro Actuators::_pyro_pcaidas_ppal(13, 39, 1000);


// ====================================================================
// Implementación de métodos
// ====================================================================

bool Actuators::init() {

    // 1. Inicializar el Buzzer
    _buzzer.init();

    // 2. Inicializar los Pirotécnicos
    _pyro_pcaidas_drogue.init();
    _pyro_pcaidas_ppal.init();

    // 3. Inicializar Servo
    _servo.init();

    return true;
}
