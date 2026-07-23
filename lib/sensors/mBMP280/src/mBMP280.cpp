//
// Created by zhl on 6/3/26.
//

#include "mBMP280.h"
#include "data.h"
#include "SerialPrint.h"
#include <Arduino.h> // Necesario para la función delay() en la calibración

mBMP280::mBMP280() : _altitud_base_m(0.0f) {}

bool mBMP280::init(uint8_t addr, uint8_t chipid) {
    if (!_bmp.begin(addr, chipid)) {
        return false;
    }

    // Configuración para telemetría de alta velocidad (Cohete)
    _bmp.setSampling(MODE_NORMAL,     // Modo Normal (medición continua)
                SAMPLING_X2,     // Sobremuestreo de Temp (Bajo, prioridad a la velocidad)
                SAMPLING_X8,     // Sobremuestreo de Presión (Moderado, reduce ruido aerodinámico)
                FILTER_OFF,      // Filtro IIR desactivado para evitar retrasos de fase en el vuelo
                STANDBY_MS_1);   // Tiempo de espera entre lecturas al mínimo (0.5 ms)

    // Calibración de la altitud en la rampa (Offset de la media)
    float suma_altitud = 0.0f;
    const int iteraciones = 200;

    for (int i = 0; i < iteraciones; i++) {
        suma_altitud += _bmp.readAltitude(1013.25f);
        // Pequeño delay para permitir que el sensor complete su conversión interna
        delay(5); 
    }

    // Guardamos la media térmica y barométrica del punto cero
    _altitud_base_m = suma_altitud / (float)iteraciones;

    return true;
}

float mBMP280::_get_altitude() {
    // Calculamos la Altitud Relativa (AGL) restando la calibración base
    return _bmp.readAltitude(1013.25f) - _altitud_base_m;
}

data_raw_bmp_t mBMP280::get_bmp_raw_data() {
    data_raw_bmp_t raw_bmp = {};

    raw_bmp.presion_hpa = _get_pressure();
    raw_bmp.temp_deg_c = _get_temperature();
    raw_bmp.altitud_snm_m = _get_altitude(); 

    return raw_bmp;
}