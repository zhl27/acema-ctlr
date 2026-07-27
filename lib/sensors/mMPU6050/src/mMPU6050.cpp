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


/*
#include "mMPU6050.h"

#include "SerialPrint.h"


// TODO: Falta revisión general de mMPU6050, especialmente en la lógica de actualización y detección de movimiento. Se recomienda implementar un Unit Test que simule movimientos significativos y menores para confirmar que el método update() solo actualiza y envía datos al plot cuando se detecta movimiento real, evitando falsos positivos por ruido o vibraciones menores.


mMPU6050::mMPU6050()
    : accelX(0.0f), accelY(0.0f), accelZ(0.0f),
      gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f), temp(0.0f),
      offset_accelX(0.0f), offset_accelY(0.0f), offset_accelZ(0.0f),
      offset_gyroX(0.0f), offset_gyroY(0.0f), offset_gyroZ(0.0f), offset_temp(0.0f) {}

bool mMPU6050::init(const uint8_t addr) {
    if (!_mpu.begin(addr)) {
        return false;
    }

    // Maximiza el rango para soportar la dinámica del cohete
    _mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // 16 G de fuerza
    _mpu.setGyroRange(MPU6050_RANGE_2000_DEG); // 2000 °/seg de aceleración angular

    // Filtro Pasa Bajos (DLPF) para mitigar vibración del motor
    _mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); // Lo configuro a fc = 10 hz, pero si hay ruido bajarlo a 5 hz
    _mpu.setSampleRateDivisor(0); // Mantiene el muestreo a 1kHz internamente

    // Configuración de Interrupciones (Opcional, según tu arquitectura actual)
    // Sirve para leer el sensor con el pin de interrupciones, no usado en la pcb
    _mpu.setInterruptPinPolarity(false);          
    _mpu.setInterruptPinLatch(false);             
    _mpu.setMotionInterrupt(false);               

    // Calibra el sensor asumiendo que está en la rampa de lanzamiento --> podemos calibrar en otras instancias --> desde GSE se podria mandar comando.
    calibrar();
    return true;
}

// NOTA IMPORTANTE: METODO BLOQUEANTE. --> se toma un tiempo para tomar muestras
int mMPU6050::calibrar() {
    constexpr int num_muestras = 1000;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    sensors_event_t a, g, t;

    // Descartar las primeras lecturas (estabilización)
    for (int i = 0; i < 100; i++) {
        _mpu.getEvent(&a, &g, &t);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Tomar N muestras
    for (int i = 0; i < num_muestras; i++) {
        _mpu.getEvent(&a, &g, &t);
        sum_ax += a.acceleration.x;
        sum_ay += a.acceleration.y;
        sum_az += a.acceleration.z;
        sum_gx += g.gyro.x;
        sum_gy += g.gyro.y;
        sum_gz += g.gyro.z;
        vTaskDelay(pdMS_TO_TICKS(5)); // cada 2 ms tomar muestras --> debe estar quieto en todo este proceso
    }

    // Promediar giroscopios (Deberían ser 0 en reposo)
    offset_gyroX = sum_gx / (float)num_muestras;
    offset_gyroY = sum_gy / (float)num_muestras;
    offset_gyroZ = sum_gz / (float)num_muestras;

    // Promediar acelerómetros
    // NOTA: El eje Y apunta hacia el cielo cuando la compu de vuelo esté instalado en el cohete.
    // Debe medir 1G (9.81 m/s^2) positivo o negativo dependiendo de la convención física.
    // Asumimos que la gravedad empuja hacia abajo, por lo que el sensor siente una aceleración normal hacia arriba de +9.81 m/s^2. --> esto se resuelve automáticamente poniendo en cero los valores de salida final de nuestra mMPU6050
    offset_accelX = (sum_ax / (float)num_muestras);
    #ifdef DEBUG_DATOS_CRUDOS
    // Para el banco de pruebas
    offset_accelY = (sum_ay / (float)num_muestras);
    offset_accelZ = (sum_az / (float)num_muestras)- 9.80665f; 
    #else
    // Para el cohete
    offset_accelY = (sum_ay / (float)num_muestras)- 9.80665f;  
    offset_accelZ = (sum_az / (float)num_muestras;
    #endif

    #ifdef DEBUG_DATOS_CRUDOS
    Serial.println("======================================================================");
    // FIX: Eliminados los '&' para pasar los valores reales y evitar corrupción de memoria
    Serial.printf("[mMPU6050 : calibrar()] BIAS ACCEL X: %f | Y: %f | Z: %f\n", offset_accelX, offset_accelY, offset_accelZ);
    Serial.printf("[mMPU6050 : calibrar()] BIAS GYRO  X: %f | Y: %f | Z: %f\n", offset_gyroX, offset_gyroY, offset_gyroZ);
    Serial.println("======================================================================");
    #endif

    return 0; // Éxito
}

data_raw_mpu_t mMPU6050::get_mpu_raw_data() {
    data_raw_mpu_t raw_mpu = {};
    sensors_event_t a, g, t;
    
    _mpu.getEvent(&a, &g, &t);

    // Aplicamos el offset calculado en la calibración
    raw_mpu.accel_x_m_s2 = a.acceleration.x - offset_accelX;
    raw_mpu.accel_y_m_s2 = a.acceleration.y - offset_accelY;
    raw_mpu.accel_z_m_s2 = a.acceleration.z - offset_accelZ;
    
    raw_mpu.gyro_x_rad_s  = g.gyro.x - offset_gyroX;
    raw_mpu.gyro_y_rad_s  = g.gyro.y - offset_gyroY;
    raw_mpu.gyro_z_rad_s  = g.gyro.z - offset_gyroZ;

    return raw_mpu;
}

*/