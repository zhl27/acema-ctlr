//
// Created by zhl on 6/25/26.
//

#include "Actuators.h"

#include "config.h"

using namespace ConfigInit;
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
mPyro Actuators::_pyro_pcaidas_drogue(
    PIRO_DROGUE_PIN,
    CONTINUIDAD_PIRO_DROGUE_PIN,
    UMBRAL_MIN_CONTINUIDAD_PIRO_mV
);

// Principal (Ppal): Activar en D13, Continuidad en VN (39), Umbral 1000
mPyro Actuators::_pyro_pcaidas_ppal(
    PIRO_PRINCIPAL_PIN, 
    CONTINUIDAD_PIRO_PRINCIPAL_PIN, 
    UMBRAL_MIN_CONTINUIDAD_PIRO_mV
);


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
