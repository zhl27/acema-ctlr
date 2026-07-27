//
// Created by zhl on 6/3/26.
//


#include "mMPU6050.h"
#include <Arduino.h>

mMPU6050::mMPU6050() 
    : m_accel_bias(math::zero()), m_gyro_bias(math::zero()) {
    m_calib_result.valid = false;
}

bool mMPU6050::init(const uint8_t addr) {
    // Nota de hardware: La dirección 0x69 está dada por el divisor de 
    // tensión a la mitad con R18 y R19 en el pin ADO.
    if (!_mpu.begin(addr)) {
        return false;
    }

    _mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    _mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    _mpu.setSampleRateDivisor(0);

    _mpu.setInterruptPinPolarity(false);          
    _mpu.setInterruptPinLatch(false);             
    _mpu.setMotionInterrupt(false);               

#ifdef SENSORES_MOCK
    _mpu.setMockSensorData(0, 9.81, 0, 0,0, 0); // en la rampa estamos quietos
#endif

    return true; 
}

mMPU6050::CalibrationStatus mMPU6050::calibrar(GravityAxis gravity_axis, int num_muestras) {
    math::RunningStatistics3D acc_stats;
    math::RunningStatistics3D gyro_stats;
    sensors_event_t a, g, t;

    // Descartar primeras muestras para estabilización
    for (int i = 0; i < 100; i++) {
        _mpu.getEvent(&a, &g, &t);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Acumular estadísticas dinámicamente usando Welford
    for (int i = 0; i < num_muestras; i++) {
        _mpu.getEvent(&a, &g, &t);
        acc_stats.push({a.acceleration.x, a.acceleration.y, a.acceleration.z});
        gyro_stats.push({g.gyro.x, g.gyro.y, g.gyro.z});
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }

    // Gyro Bias (asume 0 en reposo absoluto)
    m_gyro_bias = gyro_stats.mean();

    // Accel Bias: Construimos un vector con la gravedad esperada según la orientación física
    math::Vector3f expected_gravity = math::zero();
    constexpr float g_val = 9.80665f;
    
    switch (gravity_axis) {
        case GravityAxis::PLUS_X:  expected_gravity.x = g_val; break;
        case GravityAxis::MINUS_X: expected_gravity.x = -g_val; break;
        case GravityAxis::PLUS_Y:  expected_gravity.y = g_val; break;
        case GravityAxis::MINUS_Y: expected_gravity.y = -g_val; break;
        case GravityAxis::PLUS_Z:  expected_gravity.z = g_val; break;
        case GravityAxis::MINUS_Z: expected_gravity.z = -g_val; break;
        case GravityAxis::NONE:    break;
    }

    // El error sistemático (Bias) es la lectura media menos la gravedad teórica presente.
    m_accel_bias = acc_stats.mean() - expected_gravity;

    // Empaquetar todo en la estructura CalibrationResult
    m_calib_result.status = CalibrationStatus::Ok;
    m_calib_result.accel_bias = m_accel_bias;
    m_calib_result.gyro_bias = m_gyro_bias;
    m_calib_result.accel_variance = acc_stats.variance();
    m_calib_result.gyro_variance = gyro_stats.variance();
    m_calib_result.accel_stddev = acc_stats.stddev();
    m_calib_result.gyro_stddev = gyro_stats.stddev();
    m_calib_result.samples = acc_stats.samples();
    m_calib_result.valid = true;

    #ifdef DEBUG_DATOS_CRUDOS
    Serial.println("======================================================================");
    Serial.printf("[mMPU6050 : calibrar()] BIAS ACCEL X: %f | Y: %f | Z: %f\n", m_accel_bias.x, m_accel_bias.y, m_accel_bias.z);
    Serial.printf("[mMPU6050 : calibrar()] BIAS GYRO  X: %f | Y: %f | Z: %f\n", m_gyro_bias.x, m_gyro_bias.y, m_gyro_bias.z);
    Serial.println("======================================================================");
    #endif

    return m_calib_result.status;
}

mMPU6050::CalibrationResult mMPU6050::get_calibration_result() const {
    return m_calib_result;
}

data_raw_mpu_t mMPU6050::get_mpu_raw_data() {

#ifdef SENSORES_MOCK
    constexpr float g_val = 9.80665f;
    constexpr float a_boost = 10.0f * g_val;         // Empuje neto del motor (3G)
    constexpr unsigned long t_lanzamiento = 50000;  // Despegue en t = 50s
    constexpr unsigned long duracion_boost = 10000;  // Duración del quemado

    unsigned long t_actual = millis();

    // 1. Fase de Reposo en Rampa (0s a 50s): 1G vertical (fuerza normal del suelo hacia el cielo)
    float acc_y = g_val;

    if (t_actual > t_lanzamiento && t_actual <= (t_lanzamiento + duracion_boost)) {
        // 2. Fase Boost: 1G de normal + 2G netos de motor = 3G (+29.42 m/s²)
        // Supera el umbral de aceleración >= 2G por 0.15s para activar la MdE
        acc_y = g_val + a_boost;

    } else if (t_actual > (t_lanzamiento + duracion_boost)) {
        // 3. Fase Coast y Caída Libre (t > 53s): Motor apagado.
        // FÍSICA REAL: Un sensor MEMS en caída libre/vuelo balístico experimenta ingravidez (0G).
        acc_y = 0.0f;
    }

    // Según tus requerimientos, el eje vertical hacia el cielo es el Z.
    // Pasamos el valor al 3er parámetro: setMockSensorData(ax, ay, az, gx, gy, gz)
    _mpu.setMockSensorData(0.0f, acc_y, 0.0f, 0.0f, 0.0f, 0.0f);

    // if (t_actual % 1000 <= 100) {
    //     Serial.printf("AccY Raw: %f\n", acc_y);
    // }
#endif

    data_raw_mpu_t raw_mpu = {};
    sensors_event_t a, g, t;
    
    _mpu.getEvent(&a, &g, &t);

    // Aplicar el bias de forma unificada. Todo lo que no sea gravedad en el eje elegido, será 0.
    raw_mpu.accel_x_m_s2 = a.acceleration.x - m_accel_bias.x;
    raw_mpu.accel_y_m_s2 = a.acceleration.y - m_accel_bias.y;
    raw_mpu.accel_z_m_s2 = a.acceleration.z - m_accel_bias.z;
    
    raw_mpu.gyro_x_rad_s  = g.gyro.x - m_gyro_bias.x;
    raw_mpu.gyro_y_rad_s  = g.gyro.y - m_gyro_bias.y;
    raw_mpu.gyro_z_rad_s  = g.gyro.z - m_gyro_bias.z;

    return raw_mpu;
}