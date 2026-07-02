//
// Created by zhl on 6/25/26.
//

#include "Actuators.h"

Actuators::Actuators() {}

bool Actuators::init() {
    bool success = true;
    _buzzer.init();
    // _pyro.init(); // Add once implemented
    // _servo.init(); // Add once implemented
    return success;
}
