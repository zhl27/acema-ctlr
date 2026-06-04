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
    bool state;

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
    bool isOn() const { return state; }
};

#endif //ACEMA_CTLR_MBUZZER_H
