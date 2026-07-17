//
// Created by zhl on 6/24/26.
//

#ifndef ACEMA_CTLR_MPYRO_H
#define ACEMA_CTLR_MPYRO_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

class mPyro {
private:
    uint8_t _pinActivar;      // Pin GPIO de disparo (D12 o D13)
    uint8_t _pinContinuidad;  // Pin GPIO analógico de monitoreo (VN/39 o D35/35)
    bool _armado;             // Estado del seguro por software
    const int _umbralVoltaje_mV; // Umbral analógico para detectar continuidad

    TimerHandle_t _timer;
    static constexpr float MV_POR_PASO = 0.806f;
    /**
     * @brief Callback estático requerido por FreeRTOS para el Software Timer.
     * @param xTimer Handle del temporizador que generó el evento.
     */
    static void _timerCallback(TimerHandle_t xTimer);

    /**
     * @brief Método privado que se ejecuta automáticamente cuando el temporizador expira.
     * Desactiva el MOSFET y desarma el sistema.
     */
    void _finDisparo() const;

public:
    /**
     * @brief Constructor de la clase mPyro.
     * @param pinActivar Pin que satura el MOSFET.
     * @param pinContinuidad Pin analógico que lee el divisor de tensión.
     * @param umbralVoltaje Valor ADC mínimo (0-4095) para considerar que hay continuidad. Por defecto 1000.
     */
    mPyro(uint8_t pinActivar, uint8_t pinContinuidad, int umbralVoltaje_mV = 1000)
        : _pinActivar(pinActivar), _pinContinuidad(pinContinuidad), _armado(false), _umbralVoltaje_mV(umbralVoltaje_mV), _timer(nullptr) {}

    //mPyro();

    /**
     * @brief Destructor por si se destruye la instancia, evitando fugas de memoria en FreeRTOS.
     */
    ~mPyro();

    /**
     * @brief Configura los modos de los pines e inicializa el temporizador. Debe llamarse dentro del setup().
     */
    void init();

    /**
     * @brief Quita el seguro de software para permitir el disparo.
     */
    void armar();

    /**
     * @brief Pone el seguro de software para bloquear activaciones.
     */
    void desarmar();

    /**
     * @brief Devuelve el estado actual del seguro de software.
     */
    bool estaArmado() const;

    /**
     * @brief Comprueba si el circuito pirotécnico está cerrado (bucle intacto).
     * @return true si detecta voltaje de VBAT a través del divisor, false si está abierto o quemado.
     */
    bool tieneContinuidad();

    /**
     * @brief Realiza el disparo del canal si el sistema está armado de forma ASÍNCRONA (No bloqueante).
     * @param duracionMs Tiempo en milisegundos que el MOSFET permanecerá activo.
     * @return true si el disparo inició correctamente, false si fue rechazado (desarmado o error de timer).
     */
    bool disparar(uint32_t duracionMs = 1500);

};

#endif //ACEMA_CTLR_MPYRO_H
