#include <Arduino.h>
#include <Wire.h>
#include <SerialPrint.h> // Tu librería personalizada
#include "sensores/mBMP280.h"
#include "sensores/mGPS.h"
#include "sensores/mMPU6050.h"

const int BUZZER_PIN = 25;

mMPU6050 mpu;
mBMP280 bmp;
mGPS gps;

void setup() {
    Serial.begin(115200);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);

    Wire.begin(21, 22);
    gps.init();

    while (!Serial)
        delay(10);

    Serial.println("Adafruit MPU6050 & BMP280 test!");

    if (!mpu.init(0x69)) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");

    if (!bmp.init(0x77, BMP280_CHIPID)) {
        Serial.println("Failed to find BMP280 chip");
        while (1) { delay(10); }
    }
    Serial.println("BMP280 Found!");

    Serial.println("");

    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    delay(100);
}

void loop() {
    // Actualizar sensores
    gps.update();
    mpu.update();
    bmp.update();

    // Delay mínimo para evitar saturar el Core del ESP32 pero permitir respiro al búfer serial
    delay(1);
}