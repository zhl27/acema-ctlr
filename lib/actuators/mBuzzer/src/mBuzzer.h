//
// Created by zhl on 6/4/26.
//

#ifndef ACEMA_CTLR_MBUZZER_H
#define ACEMA_CTLR_MBUZZER_H

#include <Arduino.h>

/**
 * @class mBuzzer
 * @brief Digital Twin for a passive or active Buzzer actuator.
 *
 * Encapsulates pin configuration, basic on/off/toggle states,
 * and helper methods to emit blocking beep patterns.
 */
class mBuzzer {
private:
    int buzzerPin;
    /* Enumeracioens con nombre y apellido para evitar conflictos de nombre */
    typedef enum STATE_BUZZER { 
        BUZZER_ST_IDLE,
        BUZZER_ST_ON,
        BUZZER_ST_WAIT,
        BUZZER_ST_TOTAL
    } stBuzzer_t;

    stBuzzer_t state;
    /* Puntero al estado */
    void (mBuzzer::*_actualState)(void);
    
    /* mini timer propio. PAra no depender de RTOS*/
    uint32_t _last_ms;
    uint32_t _offTime_ms;       //  Duracion entre blink (pausas)
    uint32_t _onTime_ms;        //  Duracion del pulso ON
    uint8_t  _beepsRemaining;   //  Repeticiones (N)
    bool _reloadedSequence;     //  Flag de secuencia
   
    
    TaskHandle_t _taskHandle;
    // Tarea RTOS estática y método run interno
    static void _taskWrapper(void* pvParameters);

    /* estados */
    void stIdle(void);
    void stOn(void);
    void stWait(void);

    // Función auxiliar para cargar secuencias
    void startSequence(uint8_t repetitions, uint32_t onTime, uint32_t offTime);



public:
    /**
     * @brief Constructs the mBuzzer.
     * @param pin GPIO pin connected to the buzzer (default is 25).
     */
    mBuzzer(int pin = 25);

    /**
     * @brief Configures the GPIO pin mode.
     */
    void init();

    /**
     * @brief ejecuta la maquina de estados. Se debe llamar periodicamentre en un loop o task
     */
    void runBuzzer(){
        // Ejecuta la función del estado actual
        if (_actualState != nullptr) {
            (this->*_actualState)();
        }
    }; // Otros nombres: run; handle; runFSM; 
    
    /**
     * @brief Turns the buzzer ON.
     */
    void on();

    /**
     * @brief Turns the buzzer OFF.
     */
    void off();

    /**
     * @brief Toggles the buzzer state.
     */
    void toggle();

    /**
     * @brief Emits a single blocking beep.
     * @param durationMs Duration of the beep in milliseconds.
     */
    void beep(uint32_t durationMs);

    /**
     * @brief Plays a predefined success pattern (e.g. double beep).
     */
    void playSuccess();

    /**
     * @brief Plays a predefined error/warning pattern.
     */
    void playError();

    /**
     * @brief Returns the current state of the buzzer.
     */
    bool isOn() const { return (state == BUZZER_ST_ON); }
};

#endif //ACEMA_CTLR_MBUZZER_H
