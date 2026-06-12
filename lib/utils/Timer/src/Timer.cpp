//
// Created by zhl on 6/10/26.
//

#include "Timer.h"
#include <esp32-hal.h>

// Constructor por defecto
Timer::Timer()
  : _duration(0), _startTick(0), _expired(false), _running(false), _callback(nullptr) {}

// Constructor con callback
Timer::Timer(TimerCallback callback)
  : _duration(0), _startTick(0), _expired(false), _running(false), _callback(callback) {}

// Inicia el temporizador
bool Timer::shot(uint32_t ms) {
    if (ms == 0) return false;

    _duration = ms;
    _startTick = millis();
    _running = true;
    _expired = false;
    return true;
}

// Actualiza el estado y ejecuta el callback si expira
void Timer::update() {
    if (_running && (millis() - _startTick >= _duration)) {
        _expired = true;
        _running = false;

        if (_callback != nullptr) {
            _callback();
        }
    }
}

// Detiene el temporizador
void Timer::stop() {
    _running = false;
    _expired = false;
}

// Verifica si expiró
bool Timer::is_expired() const {
    return _expired;
}

// Verifica si está corriendo
bool Timer::is_running() const {
    return _running;
}

// Cambia el callback dinámicamente
void Timer::set_callback(TimerCallback callback) {
    _callback = callback;
}