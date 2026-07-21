//
// Created by zhl on 6/6/26.
//


#ifndef ACEMA_CTLR_DATA_H
#define ACEMA_CTLR_DATA_H

#include <cstdint>
#include <cstdio>

 
/**
 * @struct data_raw_mpu_t
 * @brief Estructura de datos crudos de la MPU6050.
 *
 * Contiene los datos sin procesar del acelerómetro y giroscopio.
 */
typedef struct {
    float accel_x_m_s2; // Aceleración X (Raw) (TODO: Ver de obtener 16 bits)
    float accel_y_m_s2; // Aceleración Y (Raw)
    float accel_z_m_s2; // Aceleración Z (Raw)
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
    float presion_hpa;  ///< Presión cruda (valor de 20 bits) --> en la libreria se usa float
    float temp_deg_c;   ///< Temperatura cruda (valor de 20 bits)
    float altitud_snm_m;    ///< Altura sobre el nivel del mar (snm)
} data_raw_bmp_t;

/**
 * @struct data_gps_t
 * @brief Estructura de datos del NEO-7M obtenidos con TinyGPS++.
 *
 * Contiene los datos del GPS.
 */
struct data_gps_t {
    bool is_valid;
    double latitude;     // Grados
    double longitude;    // Grados
    uint32_t satellites; // Cantidad de satélites visibles
    double hdop;
};

/**
 * @struct data_raw_t
 * @brief Flujo crudo. Estructura que agrupa todos los datos crudos de los sensores.
 *
 * Contenedor para los datos sin procesar de BMP280, MPU6050 y GPS.
 */
typedef struct {
    data_raw_bmp_t bmp;  ///< Datos crudos del BMP280
    data_raw_mpu_t mpu;  ///< Datos crudos del MPU6050
    data_gps_t gps;       ///< Datos crudos del GPS
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

    // --- CINEMÁTICA LINEAL (Eje Y absoluto calibrado al cielo) ---
    float altitud_filtrada_m;         // Altura filtrada sobre el suelo --> obtenida del bmp280
    float vel_z_filtrada_m_s;         // Velocidad vertical real
    float aceleracion_z_m_s2;         // Aceleración lineal absoluta (sin gravedad)

    float momentum_kg_m_s;            // Cantidad de movimiento (P = m * v)
    float temperatura_amb_c;          // Tomada estrictamente del BMP280
    float densidad_aire_kg_m3;        // Calculada por ley de gases ideales

    // --- GPS ---
    bool gps_is_valid;
    uint32_t gps_nro_satelites;
    float gps_hdop;
    float gps_latitud;
    float gps_longitud;

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

    float altura_m_snm; // obtenida del bmp280
    float altura_relativa_a_pad;

    float vel_z_bmp;
    float vel_y_bmp;
    float vel_x_bmp;

    float vel_z_mpu;
    float vel_y_mpu;
    float vel_x_mpu;

    float acel_z_mpu;

    float angulo_respecto_z;

    bool gps_is_valid;
    uint32_t gps_nro_satelites;
    float gps_hdop; // Esto muestra la precision de latitud y longitud. Menor o igual a 2 es un buen valor.
    float gps_latitud;
    float gps_longitud;

    uint32_t masa_cohete_g; // ponemos en gramos para evitar floats --> en GSE se convierte a Kg

    float angulo_airbrake;

    bool hay_continuidad_pyro_pcaidas_ppal;
    bool hay_continuidad_pyro_pcaidas_drogue;

    uint32_t estado_vuelo;
    uint32_t error_vuelo;

} data_gse_t;






#include <Arduino.h>

// ============================================================================
// 1. FUNCIONES ATÓMICAS - DATOS CRUDOS (RAW)
// ============================================================================

inline void plot_mpu_raw(const data_raw_t *data) {
    if (data == nullptr) return;
    Serial.printf(">ax_m_s2:%f\n>ay_m_s2:%f\n>az_m_s2:%f\n>gx_rad_s:%f\n>gy_rad_s:%f\n>gz_rad_s:%f\n",
                  data->mpu.accel_x_m_s2, data->mpu.accel_y_m_s2, data->mpu.accel_z_m_s2,
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s);
}

inline void plot_bmp_raw(const data_raw_t *data) {
    if (data == nullptr) return;
    Serial.printf(">presion_hpa:%f\n>temp_c:%f\n",
                  data->bmp.presion_hpa,
                  data->bmp.temp_deg_c);
}

inline void plot_gps_raw(const data_raw_t *data) {
    if (data == nullptr) return;
    Serial.printf(">lat:%ld\n>lon:%ld\n>sats:%d\n",
                  (long)data->gps.latitude,
                  (long)data->gps.longitude,
                  (int)data->gps.satellites);
}

// ============================================================================
// 2. FUNCIONES ATÓMICAS - DATOS FILTRADOS / PROCESADOS
// ============================================================================

inline void plot_cinematica_filtrada(const data_all_t *data) {
    if (data == nullptr) return;
    Serial.printf(">alt_agl_m:%.2f\n>vel_z_m_s:%.2f\n>accel_z_m_s2:%.2f\n",
                  data->altitud_filtrada_m,
                  data->vel_z_filtrada_m_s,
                  data->aceleracion_z_m_s2);
}

inline void plot_actitud_filtrada(const data_all_t *data) {
    if (data == nullptr) return;
    Serial.printf(">pitch_deg:%.2f\n>yaw_deg:%.2f\n>incl_z_deg:%.2f\n",
                  data->angulo_pitch_deg,
                  data->angulo_yaw_deg,
                  data->angulo_respecto_z_deg);
}

inline void plot_giroscopio_filtrado(const data_all_t *data) {
    if (data == nullptr) return;
    Serial.printf(">vel_ang_x_deg_s:%.2f\n>vel_ang_y_deg_s:%.2f\n>vel_ang_z_deg_s:%.2f\n",
                  data->vel_angular_x_deg_s,
                  data->vel_angular_y_deg_s,
                  data->vel_angular_z_deg_s);
}

inline void plot_ambiental_procesado(const data_all_t *data) {
    if (data == nullptr) return;
    Serial.printf(">temp_amb_c:%.2f\n>densidad_aire:%.4f\n",
                  data->temperatura_amb_c,
                  data->densidad_aire_kg_m3);
}

inline void plot_gps_procesado(const data_all_t *data) {
    if (data == nullptr) return;
    Serial.printf(">lat:%.6f\n>lon:%.6f\n>hdop:%.2f\n>sats:%d\n",
                  data->gps_latitud,
                  data->gps_longitud,
                  data->gps_hdop,
                  data->gps_nro_satelites);
}

// ============================================================================
// 3. WRAPPERS QUE ENGLOBAN TODO (ALL IN ONE)
// ============================================================================

/**
 * @brief Imprime en una sola línea del plotter todos los sensores en estado CRUDO
 */
inline void plot_all_raw(const data_raw_t *data) {
    if (data == nullptr) return;
    Serial.printf(">presion_hpa:%f\n>temp_c:%f\n>ax_m_s2:%f\n>ay_m_s2:%f\n>az_m_s2:%f\n>gx_rad_s:%f\n>gy_rad_s:%f\n>gz_rad_s:%f\n>lat:%ld\n>lon:%ld\n>sats:%d\n",
                  data->bmp.presion_hpa, data->bmp.temp_deg_c,
                  data->mpu.accel_x_m_s2, data->mpu.accel_y_m_s2, data->mpu.accel_z_m_s2,
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s,
                  (long)data->gps.latitude, (long)data->gps.longitude, (int)data->gps.satellites);
}

/**
 * @brief Imprime en una sola línea del plotter todas las variables PROCESADAS / FILTRADAS
 */
inline void plot_all_processed(const data_all_t *data) {
    if (data == nullptr) return;
    Serial.printf(">alt_agl:%.2f\n>vel_z:%.2f\n>accel_z:%.2f\n>pitch:%.2f\n>yaw:%.2f\n>incl_z:%.2f\n>vel_ang_x:%.2f\n>vel_ang_y:%.2f\n>vel_ang_z:%.2f\n>temp_amb:%.2f\n>rho:%.4f\n>lat:%.6f\n>lon:%.6f\n>hdop:%.2f\n>sats:%d\n",
                  data->altitud_filtrada_m, data->vel_z_filtrada_m_s, data->aceleracion_z_m_s2,
                  data->angulo_pitch_deg, data->angulo_yaw_deg, data->angulo_respecto_z_deg,
                  data->vel_angular_x_deg_s, data->vel_angular_y_deg_s, data->vel_angular_z_deg_s,
                  data->temperatura_amb_c, data->densidad_aire_kg_m3,
                  data->gps_latitud, data->gps_longitud, data->gps_hdop, data->gps_nro_satelites);
}


/**
 * @brief Imprime por el puerto serie todos los valores de la estructura data_raw_t.
 * * @param data Referencia constante a la estructura con los datos crudos.
 */
inline void print_data_raw(const data_raw_t *data) {
#if defined(DEBUG_DATOS_CRUDOS)
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
                  data->mpu.accel_x_m_s2, data->mpu.accel_y_m_s2, data->mpu.accel_z_m_s2);

    
    Serial.printf("[MPU6050] Gyro  X: %f | Y: %f | Z: %f\n",
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s);

    // --- Datos del GPS (nav_pvt_t) ---
    Serial.printf("[GPS]     Latitud: %ld | Longitud: %ld | Satelites: %d\n",
                  (long)data->gps.latitude,
                  (long)data->gps.longitude,
                  (int)data->gps.satellites);

    Serial.printf("==========================================\n");
    return;
#endif
}

#if defined(PLOT_MPU_ONLY) || defined(PLOT_BMP_ONLY) || defined(PLOT_GPS_ONLY) || defined(PLOT_ALL)

inline void print_plotter_data_raw(const data_raw_t *data) {
    if (data == NULL) return;

#if defined(PLOT_MPU_ONLY)
    // Ideal para calibrar offsets, ver ruido y probar el filtro complementario
    Serial.printf("AccX_g:%f,AccY_g:%f,AccZ_g:%f,"
                  "GyroX_rads:%f,GyroY_rads:%f,GyroZ_rads:%f\r\n",
                  data->mpu.accel_x_m_s2, data->mpu.accel_y_m_s2, data->mpu.accel_z_m_s2,
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s);

#elif defined(PLOT_BMP_ONLY)
    // Restamos un offset aproximado (ej. 1000 hPa) a la presión si quieres ver variaciones
    // pequeñas de altura junto con la temperatura sin que se aplasten mutuamente:
    Serial.printf("Presion_hPa:%f,Temp_C:%f\r\n",
                  data->bmp.presion_hpa,
                  data->bmp.temp_deg_c);

#elif defined(PLOT_GPS_ONLY)
    Serial.printf("Lat:%ld,Lon:%ld,Sats:%d\r\n",
                  (long)data->gps.latitude,
                  (long)data->gps.longitude,
                  (int)data->gps.satellites);

#elif defined(PLOT_ALL)
    Serial.printf("Presion:%f,Temp:%f,AccX:%f,AccY:%f,AccZ:%f,GyroX:%f,GyroY:%f,GyroZ:%f,Lat:%ld,Lon:%ld,Sats:%d\r\n",
                  data->bmp.presion_hpa, data->bmp.temp_deg_c,
                  data->mpu.accel_x_g, data->mpu.accel_y_g, data->mpu.accel_z_g,
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s,
                  (long)data->gps.lat, (long)data->gps.lon, (int)data->gps.numSV);
#endif
    return;
}

#endif


// TODO: poner "printear_data" en un lugar mejor
inline void print_data(const data_all_t *data) {
#if defined(DEBUG_DATOS_FILTRADOS)
    // Verificación de seguridad para evitar cuelgues si el puntero es nulo
    if (data == NULL) {
        Serial.printf("Error: Puntero de telemetría nulo.\n");
        return;
    }

    Serial.printf("\n=============== DATA_ALL_T (micros=%lu) ===============\n", micros());

    Serial.printf("--- CINEMÁTICA LINEAL ---\n");
    Serial.printf("Altura AGL:         %.2f m\n", data->altitud_filtrada_m);
    Serial.printf("Velocidad Z:        %.2f m/s\n", data->vel_z_filtrada_m_s);
    Serial.printf("Aceleración Z:      %.2f m/s^2\n", data->aceleracion_z_m_s2);
    // Serial.printf("Altitud Rampa ASL:  %.2f m\n", data->altitud_rampa_asl_m);

    Serial.printf("--- CINEMÁTICA ANGULAR ---\n");
    Serial.printf("Vel Angular X:      %.2f °/s (Pitch)\n", data->vel_angular_x_deg_s);
    Serial.printf("Vel Angular Y:      %.2f °/s (Roll)\n", data->vel_angular_y_deg_s);
    Serial.printf("Vel Angular Z:      %.2f °/s (Yaw)\n", data->vel_angular_z_deg_s);

    Serial.printf("--- ACTITUD ---\n");
    Serial.printf("Pitch:              %.2f °\n", data->angulo_pitch_deg);
    Serial.printf("Yaw:                %.2f °\n", data->angulo_yaw_deg);
    Serial.printf("Inclinación Z:      %.2f °\n", data->angulo_respecto_z_deg);

    Serial.printf("--- AMBIENTALES PROCESADOS ---\n");
    Serial.printf("Temperatura Amb:    %.2f °C\n", data->temperatura_amb_c);
    Serial.printf("Densidad Aire:      %.4f kg/m^3\n", data->densidad_aire_kg_m3);

    Serial.printf("--- GPS ---\n");
    if (data->gps_is_valid) {
        Serial.printf("Estado:             VÁLIDO\n");
        Serial.printf("Satélites:          %d\n", data->gps_nro_satelites);
        Serial.printf("HDOP:               %.2f\n", data->gps_hdop);
        Serial.printf("Latitud:            %.6f\n", data->gps_latitud);
        Serial.printf("Longitud:           %.6f\n", data->gps_longitud);
    } else {
        Serial.printf("Estado:             INVÁLIDO (Buscando...)\n");
    }

    Serial.printf("======================================================\n\n");
#endif
    return;
}



// Estructura del payload que viajará por la cola RTOS
struct CommandPayload {
    uint32_t opCode;
    float value;
};

#endif //ACEMA_CTLR_DATA_H



