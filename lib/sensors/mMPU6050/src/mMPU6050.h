//
// Created by zhl on 6/3/26.
//
#ifndef ACEMA_CTLR_MMPU6050_H
#define ACEMA_CTLR_MMPU6050_H

#include "data.h"
#include "../../../../src/core/math/Vector3f.h"             // API Matemática
#include "../../../../src/core/math/RunningStatistics3D.h"  // API Estadística


#ifdef  SENSORES_MOCK
    #include "mockMPU6050.h"
    using SensorMPU6050 = mockMPU6050;
#else
    #include "Adafruit_MPU6050.h"
    using SensorMPU6050 = Adafruit_MPU6050;
#endif

class mMPU6050 {
public:
    // Enum para seleccionar el eje de la gravedad dinámicamente y evitar #ifdef
    enum class GravityAxis {
        PLUS_X, MINUS_X,
        PLUS_Y, MINUS_Y,
        PLUS_Z, MINUS_Z,
        NONE
    };

    enum class CalibrationStatus {
        Ok, SensorNotFound, VehicleMoving, ExcessiveNoise, BiasTooLarge, Saturated, Timeout
    };

    struct CalibrationResult {
        CalibrationStatus status;
        math::Vector3f accel_bias;
        math::Vector3f gyro_bias;
        math::Vector3f accel_variance;
        math::Vector3f gyro_variance;
        math::Vector3f accel_stddev;
        math::Vector3f gyro_stddev;
        uint32_t samples;
        bool valid;
    };

    mMPU6050();

    bool init(uint8_t addr = 0x69);
    
    // Calibración bloqueante, inyectando el eje de la gravedad (default al cohete)
    CalibrationStatus calibrar(GravityAxis gravity_axis = GravityAxis::PLUS_Y, int num_muestras = 1000);

    data_raw_mpu_t get_mpu_raw_data();
    
    // Retorna las métricas sin romper el flujo de uso habitual
    CalibrationResult get_calibration_result() const;

private:
    SensorMPU6050 _mpu;

    math::Vector3f m_accel_bias;
    math::Vector3f m_gyro_bias;
    CalibrationResult m_calib_result;
};


/**
 * @brief Imprime el resultado de la calibración del IMU.
 * @param res Estructura de resultados de calibración.
 */
inline void print_calibration_result(const mMPU6050::CalibrationResult& res) {
#if defined(DEBUG_DATOS_CALIBRACION)
    if (!res.valid) {
        Serial.printf("Error: Resultado de calibración INVÁLIDO.\n");
        return;
    }

    Serial.printf("\n=============== CALIBRATION_RESULT (Muestras=%u) ===============\n", res.samples);
    
    Serial.printf("--- BIAS ACCEL (m/s^2) ---\n");
    Serial.printf("X: %.4f | Y: %.4f | Z: %.4f\n", res.accel_bias.x, res.accel_bias.y, res.accel_bias.z);
    
    Serial.printf("--- BIAS GYRO (rad/s) ---\n");
    Serial.printf("X: %.4f | Y: %.4f | Z: %.4f\n", res.gyro_bias.x, res.gyro_bias.y, res.gyro_bias.z);
    
    Serial.printf("--- ESTADÍSTICAS ACCEL (STDDEV) ---\n");
    Serial.printf("X: %.4f | Y: %.4f | Z: %.4f\n", res.accel_stddev.x, res.accel_stddev.y, res.accel_stddev.z);
    
    Serial.printf("--- ESTADÍSTICAS GYRO (STDDEV) ---\n");
    Serial.printf("X: %.4f | Y: %.4f | Z: %.4f\n", res.gyro_stddev.x, res.gyro_stddev.y, res.gyro_stddev.z);

    // Puedes verificar el estado si algo falló
    const char* status_str = (res.status == mMPU6050::CalibrationStatus::Ok) ? "OK" : "ERROR";
    Serial.printf("Estado final:       %s\n", status_str);

    Serial.printf("=================================================================\n\n");
#endif
}

#endif //ACEMA_CTLR_MMPU6050_H



/*
#ifndef ACEMA_CTLR_MMPU6050_H
#define ACEMA_CTLR_MMPU6050_H

#include "Adafruit_MPU6050.h"
#include "data.h"

/**
 * @class mMPU6050
 * @brief Digital Twin for the MPU6050 Accelerometer and Gyroscope sensor.
 *
 * Manages raw readings, motion interrupt configuration, and handles sending
 * plots over telemetry when motion is detected.
 */
/*
class mMPU6050 {
private:
    Adafruit_MPU6050 _mpu;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float temp;

    // Variables de calibración
    float offset_accelX;
    float offset_accelY;
    float offset_accelZ;
    float offset_gyroX;
    float offset_gyroY;
    float offset_gyroZ;
    float offset_temp;

public:
    mMPU6050();

    /**
     * @brief Initializes the sensor and configures high-pass filtering & motion detection.
     * @param addr I2C address (default 0x69)
     * @return true if initialized successfully, false otherwise.
     */

    //bool init(uint8_t addr = 0x69);

    // Getters
    // float getAccelX() const { return accelX; }
    // float getAccelY() const { return accelY; }
    // float getAccelZ() const { return accelZ; }
    // float getGyroX() const { return gyroX; }
    // float getGyroY() const { return gyroY; }
    // float getGyroZ() const { return gyroZ; }
    // float getTemp() const { return temp; }
/*
    data_raw_mpu_t get_mpu_raw_data();

    int calibrar();
};
*/

//#endif //ACEMA_CTLR_MMPU6050_H
