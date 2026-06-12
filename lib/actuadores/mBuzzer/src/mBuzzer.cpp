//
// Created by zhl on 6/4/26.
//

#include "mBuzzer.h"

mBuzzer::mBuzzer(int pin) : buzzerPin(pin), state(false) {}

void mBuzzer::init() {
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
    state = false;
}

void mBuzzer::on() {
    digitalWrite(buzzerPin, HIGH);
    state = true;
}

void mBuzzer::off() {
    digitalWrite(buzzerPin, LOW);
    state = false;
}

void mBuzzer::toggle() {
    if (state) {
        off();
    } else {
        on();
    }
}

void mBuzzer::beep(uint32_t durationMs) {
    on();
    delay(durationMs);
    off();
}

void mBuzzer::playSuccess() {
    beep(100);
    delay(50);
    beep(100);
}

void mBuzzer::playError() {
    beep(500);
    delay(100);
    beep(500);
}
