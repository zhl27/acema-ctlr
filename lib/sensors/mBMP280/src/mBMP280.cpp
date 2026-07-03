//
// Created by zhl on 6/3/26.
//

#include "mBMP280.h"

#include "data.h"
#include "SerialPrint.h"

// TODO: Crear Unit test para validar que el sensor detecta cambios significativos de presión/temperatura/altitud al soplar sobre él, y que no reacciona a cambios menores o ruido ambiental. Esto es crucial para confirmar que los umbrales definidos son adecuados para detectar el soplido sin generar falsos positivos.


// initialize
mBMP280::mBMP280()
{}

bool mBMP280::init(uint8_t addr, uint8_t chipid) {
    if (!bmp.begin(addr, chipid)) {
        return false;
    }

    // Set configuration for high filter rate and 500ms delay to capture blowing fluctuations
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);

    // // Initial readings to establish a baseline
    // last_temp = get_temperature();
    // last_pres = get_pressure();
    // last_alt  = get_pressure();
    return true;
}

data_raw_bmp_t mBMP280::get_raw_bmp() {
    data_raw_bmp_t raw_bmp = {};

    raw_bmp.presion = get_pressure();
    raw_bmp.temp = get_temperature();

    return raw_bmp;

    // // Update baseline history
    // last_temp = current_temp;
    // last_pres = current_pres;
    // last_alt  = current_alt;
}