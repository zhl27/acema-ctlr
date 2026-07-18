//
// Created by zhl on 6/25/26.
//

#include "Actuators.h"

#include "config.h"


// ====================================================================
// Inicialización de los miembros estáticos de la clase Actuators
// ====================================================================

// Inicializamos el buzzer usando el pin por defecto (25) o puedes especificar otro.
mBuzzer Actuators::_buzzer(BUZZER_PIN);

// Inicializamos el servo usando configuración específica, debemos pasarle una estructura mServoConfig_t
mServoConfig_t config_airbrake = {
    .pinServo  = SERVO_PIN,        // Pin PWM del servo
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
