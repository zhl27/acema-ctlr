//
// Created by zhl on 6/10/26.
//

#ifndef ACEMA_CTLR_TIMER_H
#define ACEMA_CTLR_TIMER_H

#include <cstdint>

/**
 * @class   Timer
 * @brief   Temporizador no bloqueante basado en milisegundos.
 */
class Timer {
public:
    // Definición del tipo de puntero a función clásico
    typedef void (*TimerCallback)();

private:
    uint32_t _duration;
    uint32_t _startTick;
    bool _expired;
    bool _running;
    TimerCallback _callback;

public:
    // Constructores
    Timer();
    Timer(TimerCallback callback);

    // Métodos principales
    bool shot(uint32_t ms);
    void update();
    void stop();

    // Getters y Setters
    bool is_expired() const;
    bool is_running() const;
    void set_callback(TimerCallback callback);
};

#endif //ACEMA_CTLR_TIMER_H