//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MMPU6050_H
#define ACEMA_CTLR_MMPU6050_H

#include "Adafruit_MPU6050.h"
#include "data.h"

/**
 * @class mMPU6050
 * @brief Digital Twin for the MPU6050 Accelerometer and Gyroscope sensor.
 *
 * Manages raw readings, motion interrupt configuration, and handles sending
 * plots over telemetry when motion is detected.
 */
class mMPU6050 {
private:
    Adafruit_MPU6050 _mpu;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float temp;

    // Variables de calibración
    float offset_accelX;
    float offset_accelY;
    float offset_accelZ;
    float offset_gyroX;
    float offset_gyroY;
    float offset_gyroZ;
    float offset_temp;

public:
    mMPU6050();

    /**
     * @brief Initializes the sensor and configures high-pass filtering & motion detection.
     * @param addr I2C address (default 0x69)
     * @return true if initialized successfully, false otherwise.
     */
    bool init(uint8_t addr = 0x69);

    // Getters
    // float getAccelX() const { return accelX; }
    // float getAccelY() const { return accelY; }
    // float getAccelZ() const { return accelZ; }
    // float getGyroX() const { return gyroX; }
    // float getGyroY() const { return gyroY; }
    // float getGyroZ() const { return gyroZ; }
    // float getTemp() const { return temp; }

    data_raw_mpu_t get_mpu_raw_data();

    int calibrar();
};


#endif //ACEMA_CTLR_MMPU6050_H
