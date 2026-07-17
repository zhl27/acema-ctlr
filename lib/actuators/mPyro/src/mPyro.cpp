//
// Created by zhl on 6/24/26.
//

#include "mPyro.h"
mPyro::mPyro(uint8_t pinActivar, uint8_t pinContinuidad, int umbralVoltaje_mV)
    : _pinActivar(pinActivar),
    _pinContinuidad(pinContinuidad), 
    _armado(false), 
    _umbralVoltaje_mV(umbralVoltaje_mV), 
    _timer(nullptr) 
{}

mPyro::~mPyro() {
    if (_timer != nullptr) {
        xTimerDelete(_timer, 0);
    }
}   


void mPyro::init() {
    pinMode(_pinActivar, OUTPUT);
    digitalWrite(_pinActivar, LOW); // Forzar estado seguro apagado al arrancar

    if (_pinContinuidad != 255) {
        pinMode(_pinContinuidad, INPUT);
    }

    // Creación del Software Timer de FreeRTOS en modo One-Shot (de un solo disparo)
    _timer = xTimerCreate(
        "mPyroTimer",        // Nombre de texto para depuración
        pdMS_TO_TICKS(1500), // Periodo inicial por defecto en ticks
        pdFALSE,             // pdFALSE = One-shot timer (se apaga automáticamente tras disparar)
        this,                // ID del timer: pasamos 'this' para recuperarlo en el callback estático
        _timerCallback       // Función estática a ejecutar al finalizar el tiempo
    );
}

void mPyro::armar() {
    _armado = true;
}

void mPyro::desarmar() {
    _armado = false;
}

bool mPyro::estaArmado() const {
    return _armado;
}

bool mPyro::tieneContinuidad() {
    if (_pinContinuidad == 255) return false;

    // Lee el valor del ADC asignado al pin S_PyRO_X
    const int tensionLectura_mV = static_cast<int>(analogRead(_pinContinuidad) * MV_POR_PASO);

    // Si la lectura supera el umbral, significa que pasa corriente desde VBAT
    // Si la lectura es inferior al umbral, significa que pasa corriente desde VBAT
    return (tensionLectura_mV < _umbralVoltaje_mV);
}

bool mPyro::disparar(uint32_t duracionMs) { // YA NO ES BLOQUEANTE: La tarea no se duerme
    if (!_armado || _timer == nullptr) {
        return false; // Rechazar disparo por seguridad si no está armado o el timer no se inicializó
    }

    digitalWrite(_pinActivar, HIGH); // Envía 3.3V al Gate del MOSFET (Cierra circuito)
    _armado = false;


    // xTimerChangePeriod actualiza el periodo y ARRANCA el temporizador si estaba detenido.
    // Un tiempo de espera de 0 (último parámetro) evita bloquear si la cola de comandos del timer está llena.
    if (xTimerChangePeriod(_timer, pdMS_TO_TICKS(duracionMs), 0) != pdPASS) {
        // Si el temporizador falla al arrancar por alguna razón, apagamos el pin inmediatamente por seguridad
        digitalWrite(_pinActivar, LOW);
        return false;
    }

    // El control retorna inmediatamente a tu Tarea/Loop principal.
    // Cuando transcurran los 'duracionMs', FreeRTOS ejecutará _timerCallback -> finDisparo().
    return true;
}


void mPyro::_timerCallback(TimerHandle_t xTimer) {
    // Recuperamos el puntero a la instancia de la clase 'mPyro' desde el ID del timer
    mPyro* instancia = static_cast<mPyro*>(pvTimerGetTimerID(xTimer));
    if (instancia != nullptr) {
        instancia->_finDisparo();
    }
}

void mPyro::_finDisparo() const {
    digitalWrite(_pinActivar, LOW); // Vuelve a poner el Gate a GND (Abre circuito)
}

