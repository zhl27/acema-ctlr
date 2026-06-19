//
// Created by zhl on 6/7/26.
//

#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include <SPI.h>

#include "globals.h"
#include "SerialPrint.h"
#include "mBuzzer.h"
#include "mBMP280.h"
#include "mGPS.h"
#include "mMPU6050.h"

#include "LoraWrapped.h"
    // Pines asignados si compilas con: pio run -e CPU-esp32
    #define LORA_SCK  18
    #define LORA_MISO 19
    #define LORA_MOSI 23
    #define LORA_CS   5
    #define LORA_RST  14
    #define LORA_DIO0 2
    #define LORA_DIO1 4
// Instanciación única y genérica usando los alias de los macros
LoraWrapped lora(LORA_CS, LORA_RST, LORA_DIO0, LORA_DIO1, SPI);

// Instantiate target components
mMPU6050 mpu;
mBMP280 bmp;
mGPS gps;
mBuzzer buzzer(BUZZER_PIN);

// Runs before every single test case
void setUp(void) {
    // Add any setup required before each test runs (optional)
}

// Runs after every single test case
void tearDown(void) {
    // Add any cleanup required after each test runs (optional)
}

// Test case 1: Verify hardware peripherals initialize properly
void test_peripheral_initialization(void) {
    buzzer.init();
    buzzer.beep(500);

    Wire.begin(WIRE_SDA, WIRE_SCL);
    gps.init();

    // Assert that the MPU6050 initializes successfully
    TEST_ASSERT_TRUE_MESSAGE(mpu.init(MPU_ADDR), "Failed to find MPU6050 chip");

    // Assert that the BMP280 initializes successfully
    TEST_ASSERT_TRUE_MESSAGE(bmp.init(BMP280_ADDR, BMP280_CHIPID), "Failed to find BMP280 chip");

    // Signal successful initialization via buzzer
    buzzer.playSuccess();
}

// Test case 2: Verify sensors handle execution updates without crashing
void test_sensor_updates(void) {
    // Basic execution check to ensure update loops don't cause panics/hangs
    gps.update();
    mpu.update();
    bmp.update();

    TEST_ASSERT_TRUE(true);
}

void setup() {
    // PlatformIO needs a small delay for the serial monitor to catch up
    delay(2000);

    SerialPrint::init(SERIAL_BAUDRATE);
    SerialPrint::msg("Starting PlatformIO Unit Tests...");

    // Initialize Unity Testing Framework
    UNITY_BEGIN();

    // Run Test Cases
    RUN_TEST(test_peripheral_initialization);
    RUN_TEST(test_sensor_updates);

    // Conclude Unity Testing
    UNITY_END();
}

void loop() {
    // Left empty for testing execution flow
}