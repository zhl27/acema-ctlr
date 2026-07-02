//
// Created by zhl on 6/25/26.
//

#include "DataFilter.h"

#include <cmath>
#include <esp_timer.h>

// Instanciación de memoria estática
float DataFilter::_h_est = 0.0f;
float DataFilter::_v_est = 0.0f;
float DataFilter::_pitch = 0.0f;
float DataFilter::_roll = 0.0f;
uint64_t DataFilter::_t_prev_us = 0;
float DataFilter::_masa_kg = 1.0f;
float DataFilter::_h_pad_offset = 0.0f;
bool DataFilter::_iniciado = false;

// TODO: Mover estos argumentos de init(...) hacia alguna config para mejor manejo.
void DataFilter::init(float masa_cohete_kg, float altitud_cero_pad_m) {
    _masa_kg = masa_cohete_kg;
    _h_pad_offset = altitud_cero_pad_m;
    _h_est = 0.0f;
    _v_est = 0.0f;
    _pitch = 0.0f;
    _roll = 0.0f;
    _t_prev_us = esp_timer_get_time(); // Reloj de hardware de alta resolución ESP32
    _iniciado = true;
}

data_all_t DataFilter::process(const data_raw_t& raw) {
    // TODO: FALTA IMPLEMENTAR CORRECTAMENTE LA LÓGICA.

    data_all_t out;

    // 1. CÁLCULO DE DELTA TIEMPO (dt)
    uint64_t now_us = raw.elapsed_time;
    float dt = (now_us - _t_prev_us) * 1e-6f; // Segundos
    if (dt <= 0.0f || !_iniciado) {
        dt = 0.0066f; // Fallback seguro a 150Hz nominal
    }
    _t_prev_us = now_us;

    // 2. AMBIENTALES Y DENSIDAD DEL AIRE (Usando estrictamente BMP280)
    out.temperatura_amb_c = raw.bmp.temp;
    float temp_k = out.temperatura_amb_c + 273.15f;
    float presion_pa = raw.bmp.presion; // Asumiendo pascales
    out.densidad_aire_kg_m3 = presion_pa / (287.058f * temp_k);

    // 3. RECHAZO DE PICOS DE PRESIÓN (Anti-Glitches sónicos)
    // Altitud barométrica calculada desde presión y temperatura BMP280.
    // Modelo hipsométrico local: h = (R * T / g) * ln(P0 / P)
    constexpr float R_AIRE_SECO_J_KG_K = 287.058f;
    constexpr float GRAVEDAD_M_S2 = 9.80665f;
    constexpr float PRESION_NIVEL_MAR_PA = 101325.0f;

    float altitud = (R_AIRE_SECO_J_KG_K * temp_k / GRAVEDAD_M_S2) *
                    std::log(PRESION_NIVEL_MAR_PA / presion_pa);

    float h_baro_raw = altitud - _h_pad_offset; // Altura relativa al pad
    float max_salto_posible = MAX_VELOCIDAD_FISICA_M_S * dt;

    if (std::fabs(h_baro_raw - _h_est) > max_salto_posible) {
        // El barómetro tiró un glitch espurio. Clampeamos al límite físico máximo:
        h_baro_raw = _h_est + std::copysign(max_salto_posible, h_baro_raw - _h_est);
    }

    // 4. FILTRO COMPLEMENTARIO DE ORIENTACIÓN
    // Aceleración lineal respecto a la gravedad
    float acc_pitch = atan2(raw.mpc.accel_y, raw.mpc.accel_z) * 57.2957795f;
    float acc_roll  = atan2(-raw.mpc.accel_x, sqrt(raw.mpc.accel_y*raw.mpc.accel_y + raw.mpc.accel_z*raw.mpc.accel_z)) * 57.2957795f;

    _pitch = ALPHA_COMP * (_pitch + raw.mpc.gyro_x * dt) + (1.0f - ALPHA_COMP) * acc_pitch;
    _roll  = ALPHA_COMP * (_roll  + raw.mpc.gyro_y * dt) + (1.0f - ALPHA_COMP) * acc_roll;

    out.pitch_deg = _pitch;
    out.roll_deg  = _roll;

    // 5. FILTRO ALPHA-BETA CINEMÁTICO (Fusión Barómetro + Acel_Z)
    // Proyección del vector aceleración hacia el vector cielo
    float cos_tilt = cos(_pitch * 0.0174533f) * cos(_roll * 0.0174533f);
    float az_cielo = (raw.mpc.accel_z * cos_tilt) - 9.81f; // Quitando gravedad terrestre

    // Predicción cinemática pura
    float h_pred = _h_est + (_v_est * dt) + (0.5f * az_cielo * dt * dt);
    float v_pred = _v_est + (az_cielo * dt);

    // Innovación (Diferencia contra el sensor físico limpiado)
    float residual = h_baro_raw - h_pred;

    // Actualización de estado
    _h_est = h_pred + (ALPHA_Z * residual);
    _v_est = v_pred + ((BETA_Z / dt) * residual);

    out.altura_m = _h_est;
    out.velocidad_z_m_s = _v_est;
    out.aceleracion_z_m_s2 = az_cielo;

    // 6. DINÁMICA Y VECTORES ROTACIONALES
    out.momentum_kg_m_s = _masa_kg * out.velocidad_z_m_s;
    out.vel_angular_x = raw.mpc.gyro_x;
    out.vel_angular_y = raw.mpc.gyro_y;
    out.vel_angular_z = raw.mpc.gyro_z;

    // Magnitud del vector omega (spin centrífugo en RPM)
    float w_norma_deg_s = sqrt(raw.mpc.gyro_x*raw.mpc.gyro_x + raw.mpc.gyro_y*raw.mpc.gyro_y + raw.mpc.gyro_z*raw.mpc.gyro_z);
    out.vel_rotacional_rpm = w_norma_deg_s * 0.166667f;

    // 7. MAPEO A ENTEROS PARA TELEMETRÍA LORA
    out.posicion_relativa = static_cast<int16_t>(std::round(_h_est));
    out.velocidad         = static_cast<int16_t>(std::round(_v_est));
    out.momentum          = static_cast<int16_t>(std::round(out.momentum_kg_m_s));

    return out;
}
