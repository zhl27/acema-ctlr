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
        Actuators() = delete;

        static mBuzzer _buzzer;
        static mServo _servo;
        static mPyro _pyro_pcaidas_drogue;
        static mPyro _pyro_pcaidas_ppal;

    public:

        /**
         * @brief Initializes all actuator drivers.
         * @return true if all initialized successfully, false otherwise.
         */
        static bool init();

        // Getters
        static mBuzzer& getBuzzer() { return _buzzer; }
        static mServo& getServo() { return _servo; }
        static mPyro& getPyroDrogue() { return _pyro_pcaidas_drogue; }
        static mPyro& getPyroPpal() { return _pyro_pcaidas_ppal; }
};


#endif //ACEMA_CTLR_ACTUATORS_H
