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

using namespace ConfigInit; 

bool Sensors::init() {
#ifdef  SENSORES_MOCK
    ESP_LOGI(TAG_TASK_SENSORS, "Inicializando Mock de Sensores.");
#else
    if(!Wire.begin(WIRE_SDA_0, WIRE_SCL_0)){
        ESP_LOGE(TAG_TASK_SENSORS, "Falló la inicialización del I2C.");
        return false;
    }
    // Wire.setClock(400000); // Set I2C clock to 400kHz Fast Mode
#endif

    if (!_mpu6050.init(MPU_ADDR)) {
        ESP_LOGE(TAG_TASK_SENSORS, "Falló la inicialización de mMPU6050.");
        return false;
    }

    // TODO: Se define desde acá si esta en banco (+Z) o en cohete (+Y)
    //_mpu6050.calibrar(mMPU6050::GravityAxis::PLUS_Z);
    ESP_LOGI(TAG_TASK_SENSORS, "mMPU6050 inicializada y calibrada!");
    //print_calibration_result(_mpu6050.get_calibration_result());
    //vTaskDelay(pdMS_TO_TICKS(3000));

    if (!_bmp280.init(BMP280_ADDR, BMP280_CHIPID)) {
        ESP_LOGE(TAG_TASK_SENSORS, "Falló la inicialización de mBMP280.");
        return false;
    }
    ESP_LOGI(TAG_TASK_SENSORS, "mBMP280 inicializada correctamente!");
    if (!_gps.init()) {
        ESP_LOGE(TAG_TASK_SENSORS, "Falló la inicialización de mGPS.");
        return false;
    }
    ESP_LOGI(TAG_TASK_SENSORS, "mGPS inicializada correctamente!");
    return true;
}

data_raw_t Sensors::get_raw_data() {
    data_raw_t raw = {};

    raw.bmp = getBMP280().get_bmp_raw_data();
    raw.mpu = getMPU6050().get_mpu_raw_data();

    raw.gps = getGPS().get_gps_raw_data();

    // La conversion de abajo deberia realizarse junto a la limpieza de los datos. Sea DataFilter o EmaFilter o afines.
    // bool fix3d      = (pvt.fixType == 3 || pvt.fixType == 4);
    // double lat      = pvt.lat * 1e-7;
    // double lon      = pvt.lon * 1e-7;
    // float alt_m     = pvt.hMSL / 1000.0f; // Altura sobre el nivel del mar
    // uint32_t sat    = pvt.numSV;

    // Timestamp con esp nativo
    raw.timestamp_us = esp_timer_get_time();

    return raw;
}