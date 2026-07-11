//
// Created by zhl on 6/25/26.
//

#include "Sensors.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG_TASK_SENSORS = "SENSORS";

// ---------------------------------------------------------
// You MUST define the static variables here so the linker
// can allocate memory for them.
// ---------------------------------------------------------
mBMP280 Sensors::_bmp280;
mGPS Sensors::_gps;
mMPU6050 Sensors::_mpu6050;

bool Sensors::init() {
    Wire.begin(WIRE_SDA_0, WIRE_SCL_0);
    // Wire.setClock(400000); // Set I2C clock to 400kHz Fast Mode
    // _gps.init(); // TODO: Reemplazar por implementación de Joe

    if (!_mpu6050.init(MPU_ADDR)) {
        ESP_LOGE(TAG_TASK_SENSORS, "Failed to find MPU6050 chip");
        return false;
    }
    ESP_LOGI(TAG_TASK_SENSORS, "MPU6050 Found!");

    if (!_bmp280.init(BMP280_ADDR, BMP280_CHIPID)) {
        ESP_LOGE(TAG_TASK_SENSORS, "Failed to find BMP280 chip");
        return false;
    }
    ESP_LOGI(TAG_TASK_SENSORS, "BMP280 Found!");
    return true;
}

// TODO: VER SI ES NECESARIO
bool Sensors::update() {
    // getGPS().update();
    return true;
}

data_raw_t Sensors::get_raw_data() {
    data_raw_t raw = {};

    raw.bmp = getBMP280().get_bmp_raw_data();
    raw.mpu = getMPU6050().get_mpu_raw_data();

    // TODO: Traer datos nav_pvt_t del módulo GPS
    raw.gps = getGPS().get_gps_raw_data();

    raw.timestamp_micros = micros(); // TODO: VER SI DEBEMOS UTILIZAR OTRA FUNCIÓN

    return raw;
}