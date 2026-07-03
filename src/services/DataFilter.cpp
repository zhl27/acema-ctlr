//
// Created by zhl on 6/25/26.
//

#include "DataFilter.h"

#include <cmath>
#include <esp_timer.h>

// --- Definición e inicialización de miembros estáticos ---
float DataFilter::_masa_cohete_kg = 15.0f;
float DataFilter::_altitud_cero_pad_m = 0.0f;
float DataFilter::_ultima_altura_m = 0.0f;
uint64_t DataFilter::_ultimo_tiempo_us = 0;
bool DataFilter::_es_primer_ciclo = true;

// Filtros para MPU6050 (puedes inicializarlos con un alfa específico si lo deseas, ej: EmaFilter(0.2f))
EmaFilter DataFilter::filter_accel_x(0.5);
EmaFilter DataFilter::filter_accel_y(0.5);
EmaFilter DataFilter::filter_accel_z(0.5);
EmaFilter DataFilter::filter_gyro_x(0.5);
EmaFilter DataFilter::filter_gyro_y(0.5);
EmaFilter DataFilter::filter_gyro_z(0.5);

// Filtros para BMP280
EmaFilter DataFilter::filter_bmp_presion(0.5);
EmaFilter DataFilter::filter_bmp_temp(0.5);

void DataFilter::init(float masa_cohete_kg, float altitud_cero_pad_m) {
    _masa_cohete_kg = masa_cohete_kg;
    _altitud_cero_pad_m = altitud_cero_pad_m;
    _es_primer_ciclo = true;
    _ultima_altura_m = 0.0f;
    _ultimo_tiempo_us = 0;

    // Configuración opcional de coeficientes Alfa si no usas los constructores por defecto
    filter_accel_x.setAlfa(0.2f);
    filter_accel_y.setAlfa(0.2f);
    filter_accel_z.setAlfa(0.2f);
    filter_gyro_x.setAlfa(0.3f);
    filter_gyro_y.setAlfa(0.3f);
    filter_gyro_z.setAlfa(0.3f);

    filter_bmp_presion.setAlfa(0.1f); // El barómetro suele requerir más filtrado (más suave)
    filter_bmp_temp.setAlfa(0.05f);
}

data_all_t DataFilter::process(const data_raw_t& raw) {
    data_all_t out;

    // ==========================================
    // STEP 1: FILTRADO DE DATOS CRUDOS (EMA)
    // ==========================================
    float raw_accel_x_f = filter_accel_x.filtrar((float)raw.mpu.accel_x);
    float raw_accel_y_f = filter_accel_y.filtrar((float)raw.mpu.accel_y);
    float raw_accel_z_f = filter_accel_z.filtrar((float)raw.mpu.accel_z);

    // Filtrar giroscopio (Cinemática Angular directa)
    out.vel_angular_x = filter_gyro_x.filtrar((float)raw.mpu.gyro_x);
    out.vel_angular_y = filter_gyro_y.filtrar((float)raw.mpu.gyro_y);
    out.vel_angular_z = filter_gyro_z.filtrar((float)raw.mpu.gyro_z);

    // Filtrar BMP280
    float presion_filtrada = filter_bmp_presion.filtrar((float)raw.bmp.presion);
    float temp_filtrada_raw = filter_bmp_temp.filtrar((float)raw.bmp.temp);


    // ==========================================
    // STEP 2: CÁLCULOS AMBIENTALES PROCESADOS
    // ==========================================
    // 1. Temperatura Ambiente (°C): Conversión típica BMP280 (Ajustar según tu driver si ya viene escalada)
    // Asumiendo que raw.bmp.temp requiere la conversión estándar, si tu driver ya la da en °C omitir división.
    out.temperatura_amb_c = temp_filtrada_raw;

    // 2. Densidad del Aire (kg/m3): Usando la Ley de Gases Ideales (P / (R * T))
    // R del aire seco = 287.05 J/(kg·K). Temperatura en Kelvin = °C + 273.15
    float temp_kelvin = out.temperatura_amb_c + 273.15f;
    // Nota: Asegúrate de que 'presion_filtrada' esté en Pascales (Pa) para esta fórmula.
    out.densidad_aire_kg_m3 = presion_filtrada / (287.05f * temp_kelvin);


    // ==========================================
    // STEP 3: CINEMÁTICA LINEAL Y DINÁMICA
    // ==========================================
    // 1. Altura (m): Conversión barométrica estándar desde presión (Pa) a metros
    // P0 estándar = 101325 Pa (o puedes usar la presión medida en el Pad durante init())
    float P0 = 101325.0f;
    float altura_absoluta = 44330.0f * (1.0f - pow((presion_filtrada / P0), 0.1902949f));
    out.altura_m = altura_absoluta - _altitud_cero_pad_m;

    // Cálculo del diferencial de tiempo (dt) para derivadas
    float dt = 0.0f;
    if (!_es_primer_ciclo && raw.elapsed_time_micros > _ultimo_tiempo_us) {
        dt = (float)(raw.elapsed_time_micros - _ultimo_tiempo_us) / 1000000.0f; // Convertir us a segundos
    }

    // 2. Velocidad Vertical Z (m/s) y Aceleración Z (m/s2)
    if (_es_primer_ciclo || dt <= 0.0f) {
        out.velocidad_z_m_s = 0.0f;
        out.aceleracion_z_m_s2 = 0.0f;
        _es_primer_ciclo = false;
    } else {
        // Velocidad vertical estimada por la derivada de la altura barométrica
        out.velocidad_z_m_s = (out.altura_m - _ultima_altura_m) / dt;

        // Aceleración lineal absoluta en Z (puedes calcularla derivando la velocidad
        // o usando el raw_accel_z_f restándole el componente de la gravedad según el pitch/roll)
        out.aceleracion_z_m_s2 = raw_accel_z_f; // Reemplazar por tu ecuación de fusión / calibración al cielo
    }

    // Guardar estados para el próximo ciclo
    _ultima_altura_m = out.altura_m;
    _ultimo_tiempo_us = raw.elapsed_time_micros;

    // 3. Momentum (P = m * v)
    out.momentum_kg_m_s = _masa_cohete_kg * out.velocidad_z_m_s;


    // ==========================================
    // STEP 4: ORIENTACIÓN Y CINEMÁTICA ANGULAR
    // ==========================================
    // 1. Magnitud escalar del spin (RPM)
    // Se calcula con la velocidad angular del eje de rotación (asumiendo Z como eje longitudinal del cohete)
    // Convertir de deg/s a RPM -> (vel * 60) / 360 = vel / 6
    out.vel_rotacional_rpm = out.vel_angular_z / 6.0f;

    // 2. Pitch y Roll (Filtro Complementario / Estimación básica con acelerómetro)
    // Nota: Esto es una estimación estática, idealmente se fusiona con el giroscopio usando el dt.
    out.pitch_deg = atan2(-raw_accel_x_f, sqrt(raw_accel_y_f * raw_accel_y_f + raw_accel_z_f * raw_accel_z_f)) * 180.0f / M_PI;
    out.roll_deg  = atan2(raw_accel_y_f, raw_accel_z_f) * 180.0f / M_PI;


    // ==========================================
    // STEP 5: TELEMETRÍA EMPAQUETADA (Cast para LoRa)
    // ==========================================
    out.posicion_relativa = (int16_t)out.altura_m;
    out.velocidad         = (int16_t)out.velocidad_z_m_s;
    out.momentum          = (int16_t)out.momentum_kg_m_s;

    return out;
}
