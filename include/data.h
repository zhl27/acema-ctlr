//
// Created by zhl on 6/6/26.
//


#ifndef ACEMA_CTLR_DATA_H
#define ACEMA_CTLR_DATA_H

#include <cstdint>
#include "UbxProtocols.h"
#include <cstdio>
#include <cstdint>

/**
 * @struct data_raw_mpu_t
 * @brief Estructura de datos crudos de la MPU6050.
 *
 * Contiene los datos sin procesar del acelerómetro y giroscopio.
 */
typedef struct {
    float accel_x_g; // Aceleración X (Raw) (TODO: Ver de obtener 16 bits)
    float accel_y_g; // Aceleración Y (Raw)
    float accel_z_g; // Aceleración Z (Raw)
    // float temp;    // Temperatura (Raw) --> No usamos el dato temp (temperatura) de la mpu5060 (mpu) porque es del chip y no del ambiente. La mpu usa la temp porque afecta a sus mediciones.
    float gyro_x_rad_s;  // Velocidad angular X (Raw)
    float gyro_y_rad_s;  // Velocidad angular Y (Raw)
    float gyro_z_rad_s;  // Velocidad angular Z (Raw)
} data_raw_mpu_t;

/**
 * @struct data_raw_bmp_t
 * @brief Estructura de datos crudos del BMP280.
 *
 * Contiene los datos sin procesar del sensor barométrico.
 */
typedef struct {
    float presion_hpa; ///< Presión cruda (valor de 20 bits) --> en la libreria se usa float
    float temp_deg_c; ///< Temperatura cruda (valor de 20 bits)
    float altitud_m;
} data_raw_bmp_t;

/**
 * @struct data_raw_t
 * @brief Flujo crudo. Estructura que agrupa todos los datos crudos de los sensores.
 *
 * Contenedor para los datos sin procesar de BMP280, MPU6050 y GPS.
 */
typedef struct {
    data_raw_bmp_t bmp;  ///< Datos crudos del BMP280
    data_raw_mpu_t mpu;  ///< Datos crudos del MPU6050
    nav_pvt_t gps;       ///< Datos crudos del GPS
    int64_t timestamp_us;  ///< Marca de tiempo de la lectura de los datos
} data_raw_t;


/**
 * @struct data_all_t
 * @brief Flujo limpiado. Estructura de datos ya "limpiados" y útiles y relevantes para la GSE y el MPC.
 *
 * @note Esta estructura se utiliza para almacenar y transmitir todas
 * las mediciones del sistema en una única entidad.
 *
 *  La decision de unificar todas las mediciones en un unico struct
 *  radica del hecho de que los datos van a venir en un flujo.
 *  Es decir, vamos a estar recibiendo datos constantemente.
 *
 *  Los subsistemas toman ese flujo de datos, y deciden qué datos del flujo les sirve.
 *
 *  Elegimos un flujo de datos, en lugar de hacer que los sensores envíen eventos, ya que de todas formas tenemos al Flash (caja negra)
 *
 */
typedef struct {
    // --- CINEMÁTICA ANGULAR (Velocidades) ---
    float vel_angular_z_deg_s;              // Yaw rate (°/s)
    float vel_angular_y_deg_s;              // Roll rate (°/s)
    float vel_angular_x_deg_s;              // Pitch rate (°/s)

    // --- CINEMÁTICA ANGULAR (Ángulos Absolutos) ---
    float angulo_pitch_deg;               // Ángulo Pitch filtrado por Kalman (°)
    float angulo_yaw_deg;                 // Ángulo Yaw filtrado por Kalman (°)
    float angulo_respecto_z_deg;          // Inclinación total del cohete


    // --- CINEMÁTICA LINEAL (Eje Z absoluto calibrado al cielo) ---
    float altitud_filtrada_m;           // Altura filtrada sobre el suelo --
    float vel_z_filtrada_m_s;            // Velocidad vertical real
    float aceleracion_z_m_s2;           // Aceleración lineal absoluta (sin gravedad)

    float momentum_kg_m_s;            // Cantidad de movimiento (P = m * v)
    float temperatura_amb_c;          // Tomada estrictamente del BMP280
    float densidad_aire_kg_m3;        // Calculada por ley de gases ideales

    // float posicion_relativa;          // Altura casteada para ahorrar ancho de banda LoRa
    // float velocidad;                  // Velocidad vertical casteada
    // float momentum;                   // Momentum casteado

    uint32_t gps_nro_satelites;
    uint32_t gps_fix_type;
    bool gps_gnss_fix_ok;
    float gps_pdop;

    float latitud;
    float longitud;

} data_all_t; ///< Información de utilidad obtenida del ambiente a través de los sensores que YA ESTÁN SANITIZADOS Y FILTRADOS!





// - Altitud Baro,
// - Altitud MPU,
// - Velocidad baro,  --> preguntar
// - Velocidad MPU,
// - Aceleración MPU,
// - Ángulo respecto a la vertical,
// - Continuidad de pirotecnicos,
// - Lat y Long,
// - Satelites,
// - Estado Actual de la MdE,
// - Ángulo Freno,
// - Código de error
typedef struct {

    float altura_bmp;
    float altura_mpu;

    float vel_z_bmp;
    float vel_y_bmp;
    float vel_x_bmp;

    float vel_z_mpu;
    float vel_y_mpu;
    float vel_x_mpu;

    float acel_z_mpu;

    float angulo_respecto_z;

    bool gps_3d_fijado;
    float latitud;
    float longitud;
    int32_t nro_satelites;


    float angulo_airbrake;

    bool hay_continuidad_pyro_pcaidas_ppal;
    bool hay_continuidad_pyro_pcaidas_drogue;

    int32_t codigo_error;

} data_gse_t;






#include <Arduino.h>

/**
 * @brief Imprime por el puerto serie todos los valores de la estructura data_raw_t.
 * * @param data Referencia constante a la estructura con los datos crudos.
 */
inline void print_data_raw(const data_raw_t *data) {
    // Verificación de seguridad para evitar cuelgues si el puntero es nulo
    if (data == NULL) {
        Serial.printf("Error: Puntero de telemetría nulo.\n");
        return;
    }
    // Encabezado con el tiempo (uint64_t usa %llu)
    Serial.printf("\n=== Datos crudos de Sensores (Tiempo: %llumicros) ===\n", data->timestamp_us);

    // --- Datos del BMP280 ---
    // Usamos %d casteando a int para los int32_t (compatible con ESP32/ARM)
    Serial.printf("[BMP280]  Presion: %f | Temp: %f\n",
                  data->bmp.presion_hpa,
                  data->bmp.temp_deg_c);

    // --- Datos del MPU6050 ---
    // Usamos %d para los int16_t (se promueven automáticamente a int en C++)
    Serial.printf("[MPU6050] Accel X: %f | Y: %f | Z: %f\n",
                  data->mpu.accel_x_g, data->mpu.accel_y_g, data->mpu.accel_z_g);

    Serial.printf("[MPU6050] Gyro  X: %f | Y: %f | Z: %f\n",
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s);

    // --- Datos del GPS (nav_pvt_t) ---
    Serial.printf("[GPS]     Latitud: %ld | Longitud: %ld | Satelites: %d\n",
                  (long)data->gps.lat,
                  (long)data->gps.lon,
                  (int)data->gps.numSV);

    Serial.printf("==========================================\n");
}

// TODO: poner "printear_data" en un lugar mejor
inline void print_data(const data_all_t *data) {
    // Verificación de seguridad para evitar cuelgues si el puntero es nulo
    if (data == NULL) {
        Serial.printf("Error: Puntero de telemetría nulo.\n");
        return;
    }

    Serial.printf("\n=============== DATA_ALL_T (micros=%lu) ===============\n", micros());

    Serial.printf("--- CINEMÁTICA LINEAL ---\n");
    Serial.printf("Altura:             %.2f m\n", data->altitud_filtrada_m);
    Serial.printf("Velocidad Z:        %.2f m/s\n", data->vel_z_filtrada_m_s);
    Serial.printf("Aceleración Z:      %.2f m/s^2\n", data->aceleracion_z_m_s2);

    Serial.printf("--- DINÁMICA ---\n");
    Serial.printf("Momentum:           %.2f kg*m/s\n", data->momentum_kg_m_s);

    Serial.printf("--- CINEMÁTICA ANGULAR ---\n");
    Serial.printf("Vel Angular X:      %.2f °/s (Pitch)\n", data->vel_angular_x_deg_s);
    Serial.printf("Vel Angular Y:      %.2f °/s (Roll)\n", data->vel_angular_y_deg_s);
    Serial.printf("Vel Angular Z:      %.2f °/s (Yaw)\n", data->vel_angular_z_deg_s);

    Serial.printf("--- AMBIENTALES PROCESADOS ---\n");
    Serial.printf("Temperatura Amb:    %.2f °C\n", data->temperatura_amb_c);
    Serial.printf("Densidad Aire:      %.4f kg/m^3\n", data->densidad_aire_kg_m3);

    Serial.printf("======================================================\n\n");
}

#endif //ACEMA_CTLR_DATA_H



