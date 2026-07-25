#include <Arduino.h>
#include "mServo.h"

mServoConfig_t configServo = {
    .pinServo = 27,     // Reemplaza por el GPIO real que usas
    .minPulse = 500,
    .maxPulse = 2500,
    .id = 1
};

mServo airbrake(&configServo);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    if(!airbrake.init()) {
        Serial.println("Error al inicializar servo (Puntero nulo)");
        while(1);
    }
    Serial.println("Servo inicializado en 0 grados.");
}

void loop() {
    Serial.println("Moviendo a 180...");
    airbrake.sendAngulo(180.0f);
    delay(2000);
    
    Serial.println("Moviendo a 5...");
    airbrake.sendAngulo(5.0f);
    delay(2000);
}