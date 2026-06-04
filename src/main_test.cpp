#include <Arduino.h>
#include <Wire.h>


#include "globals.h"
#include <actuadores/SerialPrint.h> // Tu librería personalizada
#include "actuadores/mBuzzer.h"

#include "sensores/mBMP280.h"
#include "sensores/mGPS.h"
#include "sensores/mMPU6050.h"

const int BUZZER_PIN = 25;

mMPU6050 mpu;
mBMP280 bmp;
mGPS gps;
mBuzzer buzzer(BUZZER_PIN);

void setup() {
    SerialPrint::init(SERIAL_BAUDRATE);

    // initialize buzzer using mBuzzer implementation
    buzzer.init();
    // short startup beep
    buzzer.beep(500);

    Wire.begin(21, 22);
    gps.init();

    SerialPrint::msg("Adafruit MPU6050 & BMP280 test!");

    if (!mpu.init(0x69)) {
        SerialPrint::err("Failed to find MPU6050 chip");
        while (true) { delay(10); }
    }
    SerialPrint::msg("MPU6050 Found!");

    if (!bmp.init(0x77, BMP280_CHIPID)) {
        SerialPrint::err("Failed to find BMP280 chip");
        while (true) { delay(10); }
    }
    SerialPrint::msg("BMP280 Found!");

    // two short beeps to indicate sensors found
    buzzer.beep(100);
    delay(50);
    buzzer.beep(100);

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