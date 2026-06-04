//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MBMP280_H
#define ACEMA_CTLR_MBMP280_H

#include "Adafruit_BMP280.h"

/**
 * @class mBMP280
 * @brief Digital Twin for the BMP280 Barometric Pressure and Temperature Sensor.
 *
 * Provides encapsulation of calibration, data acquisition, and noise filtering,
 * along with detecting changes and soplido (blowing) events.
 */
class mBMP280 {
private:
    Adafruit_BMP280 bmp;
    float last_temp;
    float last_pres;
    float last_alt;

    // Thresholds to filter noise and detect significant changes/blows
    const float TEMP_THRESHOLD = 0.2f;
    const float PRES_THRESHOLD = 8.0f;
    const float ALT_THRESHOLD  = 0.6f;

public:
    mBMP280();

    /**
     * @brief Initializes the BMP280 sensor and applies standard sampling settings.
     * @param addr I2C address (default 0x77)
     * @param chipid Sensor chip ID (default BMP280_CHIPID)
     * @return true if initialized successfully, false otherwise.
     */
    bool begin(uint8_t addr = 0x77, uint8_t chipid = BMP280_CHIPID);

    /**
     * @brief Performs readings, executes threshold change checks, and sends telemetries.
     */
    void update();

    // Getters for current sensor readings
    float getTemperature() const { return last_temp; }
    float getPressure() const { return last_pres; }
    float getAltitude() const { return last_alt; }
};


#endif //ACEMA_CTLR_MBMP280_H
