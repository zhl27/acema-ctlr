//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MBMP280_H
#define ACEMA_CTLR_MBMP280_H

#include "data.h"


#if defined(SENSORES_MOCK)
    #include "mockBMP280.h"
    using SensorBMP280 = mockBMP280;
#elif
    #include "Adafruit_BMP280.h"
    using SensorBMP280 = Adafruit_BMP280;
#endif


/**
 * @class mBMP280
 * @brief Digital Twin for the BMP280 Barometric Pressure and Temperature Sensor.
 *
 * Provides encapsulation of calibration, data acquisition, and noise filtering,
 * along with detecting changes and soplido (blowing) events.
 */
class mBMP280 {
private:
    SensorBMP280 _bmp;
    float _altitud_base_m; // Variable para almacenar el Offset de la rampa de lanzamiento

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
    float _get_temperature() { return _bmp.readTemperature(); }
    float _get_pressure() { return _bmp.readPressure(); }
    
    // Calcula y devuelve la altitud relativa al punto de despegue (AGL)
    float _get_altitude(); 

    data_raw_bmp_t get_bmp_raw_data();
};

#endif //ACEMA_CTLR_MBMP280_H