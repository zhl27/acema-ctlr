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

    static bool update();

    // BMP280
    // static float getBmpTemp() { return _bmp280.get_temperature(); }
    // static float getPressure() { return _bmp280.get_pressure(); }
    // // static float getAltitude() { return _bmp280.getAltitude(); }
    //
    // // GPS
    // static uint32_t getSatellites() { return _gps.getSatellites(); }
    // static double getLatitude() { return _gps.getLatitude(); }
    // static double getLongitude() { return _gps.getLongitude(); }
    // static bool is3dFixed() { return _gps.is3dFixed(); }
    //
    // // MPU6050
    // static float getAccelX() { return _mpu6050.getAccelX(); }
    // static float getAccelY() { return _mpu6050.getAccelY(); }
    // static float getAccelZ() { return _mpu6050.getAccelZ(); }
    // static float getGyroX() { return _mpu6050.getGyroX(); }
    // static float getGyroY() { return _mpu6050.getGyroY(); }
    // static float getGyroZ() { return _mpu6050.getGyroZ(); }
    // static float getMpuTemp() { return _mpu6050.getTemp(); }

    // Raw data
    static data_raw_t get_raw_data();
    // static bool update();

    // Getters for the underlying modules
    static mBMP280& getBMP280() { return _bmp280; }
    static mGPS& getGPS() { return _gps; }
    static mMPU6050& getMPU6050() { return _mpu6050; }
};

#endif //ACEMA_CTLR_SENSORS_H