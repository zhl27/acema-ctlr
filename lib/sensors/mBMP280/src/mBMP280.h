//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MBMP280_H
#define ACEMA_CTLR_MBMP280_H

#include "Adafruit_BMP280.h"
#include "data.h"

/**
 * @class mBMP280
 * @brief Digital Twin for the BMP280 Barometric Pressure and Temperature Sensor.
 *
 * Provides encapsulation of calibration, data acquisition, and noise filtering,
 * along with detecting changes and soplido (blowing) events.
 */
class mBMP280 {
private:
    Adafruit_BMP280 _bmp;
    // float last_temp;
    // float last_pres;
    // float last_alt;

public:
    mBMP280();

    /**
     * @brief Initializes the BMP280 sensor and applies standard sampling settings.
     * @param addr I2C address (default 0x77)
     * @param chipid Sensor chip ID (default BMP280_CHIPID)
     * @return true if initialized successfully, false otherwise.
     */
    bool init(uint8_t addr = 0x77, uint8_t chipid = BMP280_CHIPID);

    // Getters for current sensor readings
    float get_temperature() { return _bmp.readTemperature(); }
    float get_pressure() { return _bmp.readPressure(); }
    // float getAltitude() { return bmp.readAltitude(1013.25f); } // NO USAMOS EL VALOR CALCULADO POR EL SENSOR

    data_raw_bmp_t get_bmp_raw_data();
};


#endif //ACEMA_CTLR_MBMP280_H
