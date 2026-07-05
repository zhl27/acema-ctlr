//
// Created by zhl on 6/25/26.
//

#include "Actuators.h"


// ====================================================================
// Inicialización de los miembros estáticos de la clase Actuators
// ====================================================================

// Inicializamos el buzzer usando el pin por defecto (25) o puedes especificar otro.
mBuzzer Actuators::_buzzer(25);

// Inicializamos el servo usando su constructor por defecto.
// (Nota: Si requieres configuración específica, deberás pasar una estructura mServoConfig_t)
mServo Actuators::_servo;

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
    _pyro_pcaidas_drogue.begin();
    _pyro_pcaidas_ppal.begin();

    // 3. Inicializar Servo
    // Nota: Según el header mServo.h provisto, la clase mServo no cuenta
    // con un método 'init()' o 'begin()' independiente, ya que su
    // inicialización ocurre en el constructor parametrizado con mServoConfig_t.
    // Si más adelante se agrega un método init() a mServo, se llamaría aquí.

    return true;
}
