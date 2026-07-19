//
// Created by zhl on 6/25/26.
//

#ifndef ACEMA_CTLR_SENSORS_H
#define ACEMA_CTLR_SENSORS_H

#include "mBMP280.h"
#include "mGPS.h"
#include "mMPU6050.h"
#include "data.h"

class Sensors {
private:
    // Delete the constructor so the class cannot be instantiated
    Sensors() = delete;

    // Static hardware instances
    static mBMP280 _bmp280;
    static mGPS _gps;
    static mMPU6050 _mpu6050;

public:
    /**
     * @brief Initializes all sensor drivers.
     * @return true if all initialized successfully, false otherwise.
     */
    static bool init();

    // Raw data
    static data_raw_t get_raw_data();
    // static bool update();

    // Getters for the underlying modules
    static mBMP280& getBMP280() { return _bmp280; }
    static mGPS& getGPS() { return _gps; }
    static mMPU6050& getMPU6050() { return _mpu6050; }
};

#endif //ACEMA_CTLR_SENSORS_H