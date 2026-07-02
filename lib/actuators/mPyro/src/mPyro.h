//
// Created by zhl on 6/24/26.
//

#ifndef ACEMA_CTLR_MPYRO_H
#define ACEMA_CTLR_MPYRO_H

#include <Arduino.h>

class mPyro {
private:
    uint8_t _pinActivar;      // Pin GPIO de disparo (D12 o D13)
    uint8_t _pinContinuidad;  // Pin GPIO analógico de monitoreo (VN/39 o D35/35)
    bool _armado;             // Estado del seguro por software
    const int _umbralVoltaje; // Umbral analógico para detectar continuidad

public:
    /**
     * @brief Constructor de la clase mPyro.
     * @param pinActivar Pin que satura el MOSFET.
     * @param pinContinuidad Pin analógico que lee el divisor de tensión.
     * @param umbralVoltaje Valor ADC mínimo (0-4095) para considerar que hay continuidad. Por defecto 1000.
     */
    mPyro(uint8_t pinActivar, uint8_t pinContinuidad, int umbralVoltaje = 1000)
        : _pinActivar(pinActivar), _pinContinuidad(pinContinuidad), _armado(false), _umbralVoltaje(umbralVoltaje) {}

    /**
     * @brief Configura los modos de los pines. Debe llamarse dentro del setup().
     */
    void begin() {
        pinMode(_pinActivar, OUTPUT);
        digitalWrite(_pinActivar, LOW); // Forzar estado seguro apagado al arrancar

        if (_pinContinuidad != 255) {
            pinMode(_pinContinuidad, INPUT);
        }
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
        int lectura = analogRead(_pinContinuidad);

        // Si la lectura supera el umbral, significa que pasa corriente desde VBAT
        return (lectura > _umbralVoltaje);
    }

    /**
     * @brief Realiza el disparo del canal si el sistema está armado.
     * @param duracionMs Tiempo en milisegundos que el MOSFET permanecerá activo.
     * @return true si el disparo se ejecutó, false si fue rechazado por estar desarmado.
     */
    bool disparar(uint32_t duracionMs = 1500) {
        if (!_armado) {
            return false; // Rechazar disparo por seguridad si no está armado
        }

        digitalWrite(_pinActivar, HIGH); // Envía 3.3V al Gate del MOSFET (Cierra circuito)
        delay(duracionMs);
        digitalWrite(_pinActivar, LOW);  // Vuelve a poner el Gate a GND (Abre circuito)

        _armado = false; // Auto-desarmado inmediato tras la ignición
        return true;
    }
};


#endif //ACEMA_CTLR_MPYRO_H
