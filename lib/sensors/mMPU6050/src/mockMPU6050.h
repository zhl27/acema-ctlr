//
// Created by lucaz on 23/7/2026.
//

#ifndef ACEMA_CTLR_MOCKMPU6050_H
#define ACEMA_CTLR_MOCKMPU6050_H


#include <cstdint>

// =====================================================================
// 1. DEPENDENCIAS SIMULADAS (Adafruit Unified Sensor / Enums de MPU6050)
// =====================================================================

#ifndef _SENSORS_H
struct sensors_vec_t {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        /* Nombres alternativos según el tipo de sensor */
        struct {
            float roll;
            float pitch;
            float heading;
        };
    };
    int8_t status;
    uint8_t reserved[3];
};

struct sensors_event_t {
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    int32_t reserved0;
    int32_t timestamp;
    union {
        float           data[4];
        sensors_vec_t   acceleration;   // Para a.acceleration.x/y/z (m/s^2)
        sensors_vec_t   magnetic;
        sensors_vec_t   orientation;
        sensors_vec_t   gyro;           // Para g.gyro.x/y/z (rad/s)
        float           temperature;    // Para t.temperature (°C)
        float           distance;
        float           light;
        float           pressure;
        float           relative_humidity;
        float           current;
        float           voltage;
    };
};
#endif

// Enums simulados basados en las llamadas del usuario
enum mpu6050_accel_range_t {
    MPU6050_RANGE_2_G = 0,
    MPU6050_RANGE_4_G,
    MPU6050_RANGE_8_G,
    MPU6050_RANGE_16_G
};

enum mpu6050_gyro_range_t {
    MPU6050_RANGE_250_DEG = 0,
    MPU6050_RANGE_500_DEG,
    MPU6050_RANGE_1000_DEG,
    MPU6050_RANGE_2000_DEG
};

enum mpu6050_bandwidth_t {
    MPU6050_BAND_260_HZ = 0,
    MPU6050_BAND_184_HZ,
    MPU6050_BAND_94_HZ,
    MPU6050_BAND_44_HZ,
    MPU6050_BAND_21_HZ,
    MPU6050_BAND_10_HZ,
    MPU6050_BAND_5_HZ
};

// =====================================================================
// 2. CLASE MOCK MPU6050
// =====================================================================

class mockMPU6050 {
public:
    mockMPU6050() {
        // Inicializar datos falsos por defecto (ej. vector gravedad en Z)
        mock_accel_x = 0.0f;
        mock_accel_y = 9.81f; // Gravedad terrestre en m/s^2
        mock_accel_z = 0.0f;

        mock_gyro_x = 0.0f;
        mock_gyro_y = 0.0f;
        mock_gyro_z = 0.0f;

        mock_temp = 25.0f;    // 25 °C
    }

    // --- Métodos de Inicialización y Configuración ---

    bool begin(uint8_t addr = 0x68) {
        last_addr = addr;
        is_initialized = true;
        return true; // Simula inicialización exitosa
    }

    void setAccelerometerRange(mpu6050_accel_range_t range) {
        accel_range = range;
    }

    void setGyroRange(mpu6050_gyro_range_t range) {
        gyro_range = range;
    }

    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth) {
        filter_bandwidth = bandwidth;
    }

    void setSampleRateDivisor(uint8_t divisor) {
        sample_rate_divisor = divisor;
    }

    // --- Métodos de Configuración de Interrupciones ---

    void setInterruptPinPolarity(bool active_low) {
        int_pin_polarity = active_low;
    }

    void setInterruptPinLatch(bool held) {
        int_pin_latch = held;
    }

    void setMotionInterrupt(bool active) {
        motion_interrupt_enabled = active;
    }

    // --- Lectura de Eventos ---

    bool getEvent(sensors_event_t* accel, sensors_event_t* gyro, sensors_event_t* temp) const {
        if (!is_initialized) return false;

        // Rellenar datos falsos de Aceleración (en m/s^2)
        if (accel != nullptr) {
            accel->acceleration.x = mock_accel_x;
            accel->acceleration.y = mock_accel_y;
            accel->acceleration.z = mock_accel_z;
        }

        // Rellenar datos falsos de Giroscopio (en rad/s)
        if (gyro != nullptr) {
            gyro->gyro.x = mock_gyro_x;
            gyro->gyro.y = mock_gyro_y;
            gyro->gyro.z = mock_gyro_z;
        }

        // Rellenar datos falsos de Temperatura (en °C)
        if (temp != nullptr) {
            temp->temperature = mock_temp;
        }

        return true;
    }

    // =================================================================
    // 3. MÉTODOS AUXILIARES PARA INYECTAR DATOS EN PRUEBAS (TESTING)
    // =================================================================

    /**
     * @brief Permite inyectar lecturas sintéticas de aceleración, giroscopio
     * y temperatura para simular el comportamiento en vuelo o calibración.
     */
    void setMockSensorData(float ax, float ay, float az,
                           float gx, float gy, float gz,
                           float temperature = 25.0f) {
        mock_accel_x = ax;
        mock_accel_y = ay;
        mock_accel_z = az;
        mock_gyro_x = gx;
        mock_gyro_y = gy;
        mock_gyro_z = gz;
        mock_temp = temperature;
    }

    // Variables públicas de inspección para verificar el estado en asserts
    bool is_initialized = false;
    uint8_t last_addr = 0x00;
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
    mpu6050_bandwidth_t filter_bandwidth;
    uint8_t sample_rate_divisor = 0;
    bool int_pin_polarity = false;
    bool int_pin_latch = false;
    bool motion_interrupt_enabled = false;

private:
    float mock_accel_x, mock_accel_y, mock_accel_z;
    float mock_gyro_x, mock_gyro_y, mock_gyro_z;
    float mock_temp;
};


#endif //ACEMA_CTLR_MOCKMPU6050_H
