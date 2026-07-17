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
    const int _umbralVoltaje; // Umbral analógico para detectar continuidad

    TimerHandle_t _timer;
static constexpr float MV_POR_PASO = 0.806f;
    /**
     * @brief Callback estático requerido por FreeRTOS para el Software Timer.
     * @param xTimer Handle del temporizador que generó el evento.
     */
    static void _timerCallback(TimerHandle_t xTimer) {
        // Recuperamos el puntero a la instancia de la clase 'mPyro' desde el ID del timer
        mPyro* instancia = static_cast<mPyro*>(pvTimerGetTimerID(xTimer));
        if (instancia != nullptr) {
            instancia->_finDisparo();
        }
    }

    /**
     * @brief Método privado que se ejecuta automáticamente cuando el temporizador expira.
     * Desactiva el MOSFET y desarma el sistema.
     */
    void _finDisparo() const {
        digitalWrite(_pinActivar, LOW); // Vuelve a poner el Gate a GND (Abre circuito)
    }

public:
    /**
     * @brief Constructor de la clase mPyro.
     * @param pinActivar Pin que satura el MOSFET.
     * @param pinContinuidad Pin analógico que lee el divisor de tensión.
     * @param umbralVoltaje Valor ADC mínimo (0-4095) para considerar que hay continuidad. Por defecto 1000.
     */
    mPyro(uint8_t pinActivar, uint8_t pinContinuidad, int umbralVoltaje = 1000)
        : _pinActivar(pinActivar), _pinContinuidad(pinContinuidad), _armado(false), _umbralVoltaje(umbralVoltaje), _timer(nullptr) {}

    mPyro();

    /**
     * @brief Destructor por si se destruye la instancia, evitando fugas de memoria en FreeRTOS.
     */
    ~mPyro() {
        if (_timer != nullptr) {
            xTimerDelete(_timer, 0);
        }
    }

    /**
     * @brief Configura los modos de los pines e inicializa el temporizador. Debe llamarse dentro del setup().
     */
    void init() {
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

    /**
     * @brief Quita el seguro de software para permitir el disparo.
     */
    void armar() {
        _armado = true;
    }

    /**
     * @brief Pone el seguro de software para bloquear activaciones.
     */
    void desarmar() {
        _armado = false;
    }

    /**
     * @brief Devuelve el estado actual del seguro de software.
     */
    bool estaArmado() const {
        return _armado;
    }

    /**
     * @brief Comprueba si el circuito pirotécnico está cerrado (bucle intacto).
     * @return true si detecta voltaje de VBAT a través del divisor, false si está abierto o quemado.
     */
    bool tieneContinuidad() {
        if (_pinContinuidad == 255) return false;

        // Lee el valor del ADC asignado al pin S_PyRO_X
        const int tensionLectura_mV = static_cast<int>(analogRead(_pinContinuidad) * MV_POR_PASO);

        // Si la lectura supera el umbral, significa que pasa corriente desde VBAT
// Si la lectura es inferior al umbral, significa que pasa corriente desde VBAT
return (tensionLectura_mV < _umbralVoltaje_mV);
    }

    /**
     * @brief Realiza el disparo del canal si el sistema está armado de forma ASÍNCRONA (No bloqueante).
     * @param duracionMs Tiempo en milisegundos que el MOSFET permanecerá activo.
     * @return true si el disparo inició correctamente, false si fue rechazado (desarmado o error de timer).
     */
    bool disparar(uint32_t duracionMs = 1500) { // YA NO ES BLOQUEANTE: La tarea no se duerme
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

};

#endif //ACEMA_CTLR_MPYRO_H
