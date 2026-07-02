//
// Created by zhl on 6/25/26.
//

#include "Sensors.h"

Sensors::Sensors() {}

bool Sensors::init() {
    bool success = true;
    if (!_bmp280.init()) success = false;
    _gps.init();
    if (!_mpu6050.init()) success = false;
    return success;
}

data_raw_t Sensors::getRawData() const {
    data_raw_t raw = {};
    
    // Fill BMP280 raw (or processed as fallback) data
    raw.bmp.presion = static_cast<int32_t>(_bmp280.getPressure());
    raw.bmp.temp = static_cast<int32_t>(_bmp280.getTemperature());
    
    // Fill MPU6050 raw (or processed as fallback) data
    raw.mpc.accel_x = static_cast<int16_t>(_mpu6050.getAccelX());
    raw.mpc.accel_y = static_cast<int16_t>(_mpu6050.getAccelY());
    raw.mpc.accel_z = static_cast<int16_t>(_mpu6050.getAccelZ());
    raw.mpc.gyro_x = static_cast<int16_t>(_mpu6050.getGyroX());
    raw.mpc.gyro_y = static_cast<int16_t>(_mpu6050.getGyroY());
    raw.mpc.gyro_z = static_cast<int16_t>(_mpu6050.getGyroZ());
    raw.mpc.temp = static_cast<int16_t>(_mpu6050.getTemp());
    
    // For GPS, we don't have a direct way to fill nav_pvt_t from TinyGPSPlus easily.
    // This requires a different GPS driver/parser.
    
    raw.elapsed_time = 0; // Needs a real timestamp implementation
    
    return raw;
}
