//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MMPU6050_H
#define ACEMA_CTLR_MMPU6050_H

#include "Adafruit_MPU6050.h"

/**
 * @class mMPU6050
 * @brief Digital Twin for the MPU6050 Accelerometer and Gyroscope sensor.
 *
 * Manages raw readings, motion interrupt configuration, and handles sending
 * plots over telemetry when motion is detected.
 */
class mMPU6050 {
private:
    Adafruit_MPU6050 mpu;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float temp;

public:
    mMPU6050();

    /**
     * @brief Initializes the sensor and configures high pass filtering & motion detection.
     * @param addr I2C address (default 0x69)
     * @return true if initialized successfully, false otherwise.
     */
    bool begin(uint8_t addr = 0x69);

    /**
     * @brief Updates internal sensor state. If motion interrupt is active, publishes plots.
     */
    void update();

    // Getters
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    float getTemp() const { return temp; }
};


#endif //ACEMA_CTLR_MMPU6050_H
