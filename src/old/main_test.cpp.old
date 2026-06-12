#include <Arduino.h>
#include <Wire.h>


#include "globals.h"

#include <utils/SerialPrint.h>
#include "actuadores/mBuzzer.h"

#include "sensores/mBMP280.h"
#include "sensores/mGPS.h"
#include "sensores/mMPU6050.h"


mMPU6050 mpu;
mBMP280 bmp;
mGPS gps;
mBuzzer buzzer(BUZZER_PIN);

void setup() { // seteamos variables y constantes iniciales
    SerialPrint::init(SERIAL_BAUDRATE);

    // initialize buzzer using mBuzzer implementation
    buzzer.init();
    // short startup beep
    buzzer.beep(500);

    Wire.begin(WIRE_SDA, WIRE_SCL);
    gps.init();

    SerialPrint::msg("Adafruit MPU6050 & BMP280 test!");

    if (!mpu.init(MPU_ADDR)) {
        SerialPrint::err("Failed to find MPU6050 chip");
        while (true) { delay(10); }
    }
    SerialPrint::msg("MPU6050 Found!");

    if (!bmp.init(BMP280_ADDR, BMP280_CHIPID)) {
        SerialPrint::err("Failed to find BMP280 chip");
        while (true) { delay(10); }
    }
    SerialPrint::msg("BMP280 Found!");

    // two short beeps to indicate sensors found
    buzzer.playSuccess();

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