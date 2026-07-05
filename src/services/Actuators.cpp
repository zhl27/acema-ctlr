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
mServoConfig_t config_servo = { // nota para el lector: esto es equivalente c/c++ a la solucion del code smell long parameters en Diseño de Sistemas
    .pinServo = 0,
    .pinPot = 0,
    .pendiente = 0,
    .alpha = 0,
    .deadband = 0,
    .minPulse = 0,
    .maxPulse = 0,
    .id = 0
};
mServo Actuators::_servo(&config_servo);

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
