//
// Created by zhl on 6/4/26.
//

#include "mBuzzer.h"

mBuzzer::mBuzzer(int pin) :
    buzzerPin(pin),
    state(false)
{}

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

// TODO: BUZZER ES BLOQUEANTE, AHORA MISMO SOLAMENTE SOLAMENTE SE PUEDE USAR EN EL SETUP
void mBuzzer::beep(uint32_t durationMs) {
    on();
    vTaskDelay(pdMS_TO_TICKS(durationMs));
    off();
}

void mBuzzer::playSuccess() {
    beep(100);
    vTaskDelay(pdMS_TO_TICKS(50));
    beep(100);
}

void mBuzzer::playError() {
    beep(500);
    vTaskDelay(pdMS_TO_TICKS(100));
    beep(500);
}
