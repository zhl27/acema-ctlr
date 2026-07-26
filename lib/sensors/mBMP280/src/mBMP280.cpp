//
// Created by zhl on 6/3/26.
//

#include "mBMP280.h"
#include "data.h"
#include "SerialPrint.h"
#include <Arduino.h> // Necesario para la función delay() en la calibración

// mBMP280::mBMP280() : _altitud_base_m(0.0f) {}
mBMP280::mBMP280() {};

bool mBMP280::init(uint8_t addr, uint8_t chipid) {
    if (!_bmp.begin(addr, chipid)) {
        return false;
    }

#ifdef SENSORES_MOCK
    // Configuración para telemetría de alta velocidad (Cohete)
    _bmp.setSampling(MODE_NORMAL,     // Modo Normal (medición continua)
                     SAMPLING_X2,     // Sobremuestreo de Temp (Bajo, prioridad a la velocidad)
                     SAMPLING_X8,     // Sobremuestreo de Presión (Moderado, reduce ruido aerodinámico)
                     FILTER_OFF,      // Filtro IIR desactivado para evitar retrasos de fase en el vuelo
                     STANDBY_MS_1);   // Tiempo de espera entre lecturas al mínimo (0.5 ms)

    _bmp.setMockAltitude(1000.25f);
#else
    // Configuración para telemetría de alta velocidad (Cohete)
    _bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Modo Normal (medición continua)
                     Adafruit_BMP280::SAMPLING_X2,     // Sobremuestreo de Temp (Bajo, prioridad a la velocidad)
                     Adafruit_BMP280::SAMPLING_X8,     // Sobremuestreo de Presión (Moderado, reduce ruido aerodinámico)
                     Adafruit_BMP280::FILTER_OFF,      // Filtro IIR desactivado para evitar retrasos de fase en el vuelo
                     Adafruit_BMP280::STANDBY_MS_1);   // Tiempo de espera entre lecturas al mínimo (0.5 ms)
#endif




    return true;
}

float mBMP280::_get_altitude() {
    // Calculamos la Altitud Relativa (AGL) restando la calibración base
    // return _bmp.readAltitude(1013.25f) - _altitud_base_m;
    // Obtenemos la altura directamente.
    return _bmp.readAltitude(1013.25f);
}

/*
 * Bloqueante
 *
 * Lo utilizamos para obtener la altitud del pad.
 *
 */
float mBMP280::get_altitude_media_iterations(const int nro_iteraciones) {
    // Calibración de la altitud en la rampa (Offset de la media)
    float suma_altitud = 0.0f;

    // Fase de acumulación: Promedia la altitud antes del vuelo
    for (int i = 0; i < nro_iteraciones; i++) {
        suma_altitud += _bmp.readAltitude(1013.25f);
        // Pequeño delay para permitir que el sensor complete su conversión interna
        delay(5);
    }

    // Retornamos la media térmica y barométrica del punto cero
    return suma_altitud / static_cast<float>(nro_iteraciones);
}


data_raw_bmp_t mBMP280::get_bmp_raw_data() {

#ifdef SENSORES_MOCK
    // 1. CONSTANTES FÍSICAS Y DEL SISTEMA (Únicas fuentes de verdad)
    constexpr float g_val = 9.80665f;
    constexpr float a_boost = 2.0f * g_val;         // Aceleración neta hacia arriba del motor (2G)
    constexpr float rampa_asl = 100.0f;             // Altura cruda de la rampa sobre nivel del mar
    constexpr unsigned long t_lanzamiento = 50000;  // Despegue en t = 50s (en ms)
    constexpr unsigned long duracion_boost = 10000;  // Duración del quemado

    // 2. DERIVACIÓN CINEMÁTICA ANALÍTICA AL CORTE DE MOTOR
    // Se calculan en tiempo de compilación integrando la aceleración constante:
    // v(t) = v_0 + a*t   |   h(t) = h_0 + v_0*t + 0.5*a*t^2
    constexpr float t_boost_seg = duracion_boost / 1000.0f;
    constexpr float v_corte = a_boost * t_boost_seg;                             // Vel. al apagar motor (~58.84 m/s)
    constexpr float delta_h_corte = 0.5f * a_boost * (t_boost_seg * t_boost_seg); // Altura ganada en quema (~88.26 m)
    constexpr float h_corte = rampa_asl + delta_h_corte;                         // Altura absoluta al corte (~188.26 m)

    const unsigned long t_actual = millis();
    float alt_cruda = rampa_asl;                    // Valor por defecto en reposo

    if (t_actual <= t_lanzamiento) {
        // FASE 1: Reposo en rampa (a_neto = 0 respecto al suelo -> v = 0 -> h = rampa_asl)
        alt_cruda = rampa_asl;

    } else if (t_actual <= (t_lanzamiento + duracion_boost)) {
        // FASE 2: Propulsión / Boost (50s a 53s)
        // Partimos del reposo (v_0 = 0, h_0 = rampa_asl). Solo actúa a_boost hacia arriba:
        // h(t) = h_rampa + 0.5 * a_boost * t^2
        float t_rel = (t_actual - t_lanzamiento) / 1000.0f; // laburamos con segundos
        alt_cruda = rampa_asl + (0.5f * a_boost * (t_rel * t_rel));

    } else {
        // FASE 3: Vuelo libre balístico (t > 53s)
        // El motor se apaga; la única aceleración actuando es la gravedad (-g_val).
        // Usamos como condiciones iniciales la velocidad y altitud exactas donde terminó el boost:
        // h(t) = h_corte + (v_corte * t_caida) - (0.5 * g * t_caida^2)
        float t_caida = (t_actual - (t_lanzamiento + duracion_boost)) / 1000.0f;
        alt_cruda = h_corte + (v_corte * t_caida) - (0.5f * g_val * (t_caida * t_caida));

        // Protección física: el cohete no puede penetrar por debajo de la rampa al caer
        if (alt_cruda < rampa_asl) {
            alt_cruda = rampa_asl;
        }
    }

    // Inyectamos el dato CRUDO al sensor BMP280.
    _bmp.setMockAltitude(alt_cruda);
#endif

    data_raw_bmp_t raw_bmp = {};

    raw_bmp.presion_hpa = _get_pressure();
    raw_bmp.temp_deg_c = _get_temperature();
    raw_bmp.altitud_snm_m = _get_altitude(); 

    return raw_bmp;
}