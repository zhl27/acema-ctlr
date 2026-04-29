//
// Created by lucaz on 31/1/2026.
//

#include "FlightController.h"


void FlightController::update() {
    if (serial.available()) {
        char c = serial.read();

        if (c == '1') led.on();
        if (c == '0') led.off();
    }
}