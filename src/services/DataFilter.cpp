//
// Created by zhl on 6/25/26.
//

#include "DataFilter.h"

#include <cmath>
#include <esp_timer.h>

#include "core/mde_cohete/include.h"


// --- Definición e inicialización de miembros estáticos ---
// float DataFilter::_masa_cohete_kg = 15.0f;
// float DataFilter::_altitud_cero_pad_m = 0.0f;
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

void DataFilter::init() {
    // _masa_cohete_kg = masa_cohete_kg; // hacerlos valores de COHETE
    // _altitud_cero_pad_m = altitud_cero_pad_m;
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
    // 1. Temperatura Ambiente (°C): Conversión típica BMP280 (Ajustar según driver si ya viene escalada)
    // Asumiendo que raw.bmp.temp requiere la conversión estándar, si el driver ya la da en °C omitir división.
    out.temperatura_amb_c = temp_filtrada_raw;

    // 2. Densidad del Aire (kg/m3): Usando la Ley de Gases Ideales (P / (R * T))
    // R del aire seco = 287.05 J/(kg·K). Temperatura en Kelvin = °C + 273.15
    const float temp_kelvin = out.temperatura_amb_c + 273.15f;
    // La presión debe estar en Pascales (Pa)
    out.densidad_aire_kg_m3 = presion_filtrada / (287.05f * temp_kelvin);


    // ==========================================
    // STEP 3: CINEMÁTICA LINEAL Y DINÁMICA
    // ==========================================
    // 1. Altura (m): Conversión barométrica estándar desde presión (Pa) a metros
    float P0 = 101325.0f; // Idealmente esto se calibra en el Pad // TODO: crear variable COHETE.presion_en_pad
    float altura_absoluta = 44330.0f * (1.0f - pow((presion_filtrada / P0), 0.1902949f));
    out.altura_m = altura_absoluta - Cohete::SYSTEM.altitud_cero_pad_m;

    // Cálculo del diferencial de tiempo (dt) para derivadas
    float dt = 0.0f;
    if (!_es_primer_ciclo && raw.elapsed_time_micros > _ultimo_tiempo_us) {
        dt = static_cast<float>(raw.elapsed_time_micros - _ultimo_tiempo_us) / 1000000.0f; // Convertir us a segundos
    }

    // 2. Velocidad Vertical Z (m/s) y Aceleración Z (m/s2)
    if (_es_primer_ciclo || dt <= 0.0f) {
        out.velocidad_z_m_s = 0.0f;
        out.aceleracion_z_m_s2 = 0.0f;
        _es_primer_ciclo = false;
    } else {
        // Velocidad vertical estimada por la derivada de la altura barométrica
        out.velocidad_z_m_s = (out.altura_m - _ultima_altura_m) / dt;

        // Aceleración lineal filtrada
        // NOTA: Para tener la aceleración absoluta sin gravedad, se debe restar g (~9.81)
        // multiplicada por el coseno del ángulo respecto a Z.
        out.aceleracion_z_m_s2 = raw_accel_z_f;
    }

    // Guardar estados para el próximo ciclo
    _ultima_altura_m = out.altura_m;
    _ultimo_tiempo_us = raw.elapsed_time_micros;

    // 3. Momentum (P = m * v)
    out.momentum_kg_m_s = Cohete::SYSTEM.masa_cohete_kg * out.velocidad_z_m_s;


    // ==========================================
    // STEP 4: ORIENTACIÓN Y CINEMÁTICA ANGULAR
    // ==========================================

    // Obtenemos la norma del vector aceleración para sacar el ángulo de desviación
    float norm_accel = sqrt(raw_accel_x_f * raw_accel_x_f +
                            raw_accel_y_f * raw_accel_y_f +
                            raw_accel_z_f * raw_accel_z_f);

    // Ángulo respecto al eje Z (Hacia el cielo)
    // Se usa el arccos( Z / Norma ). Retorna en Radianes y se convierte a Grados.
    if (norm_accel > 0.0f) {
        out.angulo_respecto_z = acos(raw_accel_z_f / norm_accel) * 180.0f / M_PI;
    } else {
        out.angulo_respecto_z = 0.0f;
    }


    // ==========================================
    // STEP 5: INTEGRACIÓN DE DATOS DEL GPS
    // ==========================================
    // Nota: Los atributos raw.gps.* corresponden a los estándares de u-blox UBX-NAV-PVT
    // Si tu librería tiene nombres ligeramente diferentes (ej. num_sv), deberás ajustarlo.

    out.gps_nro_satelites = raw.gps.numSV;
    out.gps_fix_type      = raw.gps.fixType;

    // Acceso directo a la estructura de bits definida en tu union
    out.gps_gnss_fix_ok   = raw.gps.flags.bits.gnssFixOK;

    out.gps_pdop          = static_cast<float>(raw.gps.pDOP) * 0.01f; // pDOP llega como entero escalado, se multiplica por 0.01 según tu comentario

    // Latitud y longitud se envían multiplicadas por 1e7 para entrar en enteros de 32 bits
    out.latitud           = static_cast<float>(raw.gps.lat) * 1e-7f;
    out.longitud          = static_cast<float>(raw.gps.lon) * 1e-7f;


    return out;
}
