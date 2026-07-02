//
// Created by zhl on 6/25/26.
//

#ifndef ACEMA_CTLR_ACTUATORS_H
#define ACEMA_CTLR_ACTUATORS_H


#include <cstdint>

#include "mBuzzer.h"
#include "mPyro.h"
#include "mServo.h"

class Actuators {
    private:
        mBuzzer _buzzer;
        mPyro _pyro;
        mServo _servo;

    public:
        Actuators();

        /**
         * @brief Initializes all actuator drivers.
         * @return true if all initialized successfully, false otherwise.
         */
        bool init();

        // Getters
        mBuzzer& getBuzzer() { return _buzzer; }
        mPyro& getPyro() { return _pyro; }
        mServo& getServo() { return _servo; }

        // Declarative high-level accessors
        // Buzzer
        void beepBuzzer(uint32_t durationMs) { _buzzer.beep(durationMs); }
        void buzzerOn() { _buzzer.on(); }
        void buzzerOff() { _buzzer.off(); }
        void playSuccessSound() { _buzzer.playSuccess(); }
        void playErrorSound() { _buzzer.playError(); }
};


#endif //ACEMA_CTLR_ACTUATORS_H
