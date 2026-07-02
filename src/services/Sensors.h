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
        mBMP280 _bmp280;
        mGPS _gps;
        mMPU6050 _mpu6050;

    public:

        Sensors();
        /**
         * @brief Initializes all sensor drivers.
         * @return true if all initialized successfully, false otherwise.
         */
        bool init();

        // BMP280
        float getTemperature() const { return _bmp280.getTemperature(); }
        float getPressure() const { return _bmp280.getPressure(); }
        float getAltitude() const { return _bmp280.getAltitude(); }

        // GPS
        uint32_t getSatellites() { return _gps.getSatellites(); }
        double getLatitude() { return _gps.getLatitude(); }
        double getLongitude() { return _gps.getLongitude(); }
        bool isLocationValid() const { return _gps.isLocationValid(); }

        // MPU6050
        float getAccelX() const { return _mpu6050.getAccelX(); }
        float getAccelY() const { return _mpu6050.getAccelY(); }
        float getAccelZ() const { return _mpu6050.getAccelZ(); }
        float getGyroX() const { return _mpu6050.getGyroX(); }
        float getGyroY() const { return _mpu6050.getGyroY(); }
        float getGyroZ() const { return _mpu6050.getGyroZ(); }
        float getMpuTemp() const { return _mpu6050.getTemp(); }

        // Raw data
        data_raw_t getRawData() const;

        // Getters
        mBMP280& getBMP280() { return _bmp280; }
        mGPS& getGPS() { return _gps; }
        mMPU6050& getMPU6050() { return _mpu6050; }
};


#endif //ACEMA_CTLR_SENSORS_H
