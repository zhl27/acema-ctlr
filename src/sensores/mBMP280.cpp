//
// Created by zhl on 6/3/26.
//

#include "mBMP280.h"

#include "../utils/SerialPrint.h"

// initialize
mBMP280::mBMP280()
    : last_temp(0.0f), last_pres(0.0f), last_alt(0.0f) {}

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

    // Initial readings to establish a baseline
    last_temp = bmp.readTemperature();
    last_pres = bmp.readPressure();
    last_alt  = bmp.readAltitude(1013.25f);
    return true;
}

void mBMP280::update() {
    float current_temp = bmp.readTemperature();
    float current_pres = bmp.readPressure();
    float current_alt  = bmp.readAltitude(1013.25f);

    // Use absolute differences to evaluate significant changes
    bool temp_changed = abs(current_temp - last_temp) >= TEMP_THRESHOLD;
    bool pres_changed = abs(current_pres - last_pres) >= PRES_THRESHOLD;
    bool alt_changed  = abs(current_alt - last_alt)   >= ALT_THRESHOLD;

    if (temp_changed || pres_changed || alt_changed) {
        // If the pressure or altitude changed, it indicates blowing on the sensor
        if (pres_changed || alt_changed) {
            SerialPrint::msg("¡Soplido detectado en el BMP280!");
        }

        SerialPrint::plot("bmp_temp", static_cast<float>(current_temp));
        SerialPrint::plot("bmp_pres", static_cast<float>(current_pres));
        SerialPrint::plot("bmp_alt",  static_cast<float>(current_alt));

        // Update baseline history
        last_temp = current_temp;
        last_pres = current_pres;
        last_alt  = current_alt;
    }
}