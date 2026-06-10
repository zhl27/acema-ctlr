//
// Created by zhl on 6/10/26.
//

#ifndef ACEMA_CTLR_TIMER_H
#define ACEMA_CTLR_TIMER_H


#include <cstdint>
#include <esp32-hal.h>

/**
 * @class   Timer
 * @brief   Temporizador no bloqueante basado en milisegundos.
 * @details Utiliza aritmética de resta para ser inmune al desbordamiento de millis() cada 49 días.
 */
class Timer {
private:
    uint32_t _duration;
    uint32_t _startTick;
    bool _expired;
    bool _running;

public:
    /**
       * @brief Constructor del temporizador.
       */
    Timer()
      : _duration(0), _startTick(0), _expired(false), _running(false) {}

    /**
       * @brief Inicia o reinicia el temporizador.
       * @param ms Tiempo en milisegundos.
       * @return true si el tiempo es válido, false si es 0.
       */
    bool shot(uint32_t ms) {
        if (ms == 0) return false;

        _duration = ms;
        _startTick = millis();
        _running = true;
        _expired = false;
        return true;
    }

    /**
       * @brief Actualiza el estado del temporizador. Debe llamarse en cada iteración del loop o HSM.
       */
    void update() {
        if (_running && (millis() - _startTick >= _duration)) {
            _expired = true;
            _running = false;
        }
    }

    /**
       * @brief Verifica si el tiempo ha transcurrido.
       * @return true si el tiempo se cumplió.
       */
    bool is_expired() const {
        return _expired;
    }

    /**
       * @brief Indica si el temporizador está contando actualmente.
       */
    bool is_running() const {
        return _running;
    }

    /**
       * @brief Detiene el temporizador y limpia los estados.
       */
    void stop() {
        _running = false;
        _expired = false;
    }
};


#endif //ACEMA_CTLR_TIMER_H
