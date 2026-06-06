//
// Created by zhl on 6/6/26.
//

#ifndef ACEMA_CTLR_DATA_H
#define ACEMA_CTLR_DATA_H

/************************/
/*     DATOS CRUDOS     */
/************************/

/**
 * @struct data_raw_mpc_t
 * @brief Estructura de datos crudos de la MPU6050.
 *
 * Contiene los datos sin procesar del acelerómetro y giroscopio.
 */
typedef struct {
    float altitud;      ///< Altitud en metros
    float velocidad;    ///< Velocidad en m/s
    float posicion;     ///< Posición en metros
} data_raw_mpc_t;

/**
 * @struct data_raw_bmp_t
 * @brief Estructura de datos crudos del BMP280.
 *
 * Contiene los datos sin procesar del sensor barométrico.
 */
typedef struct {
    float presion;          ///< Presión en hPa
    float temperatura_bmp;  ///< Temperatura en °C
    float altitud;          ///< Altitud en metros
} data_raw_bmp_t;

/**
 * @struct data_raw_gps_t
 * @brief Estructura de datos crudos del GPS.
 *
 * Contiene los datos sin procesar del receptor GPS.
 */
typedef struct {
    double latitud;     ///< Latitud en grados
    double longitud;    ///< Longitud en grados
    uint32_t satelites; ///< Número de satélites detectados
    bool gps_valido;    ///< Indicador de validez del GPS
} data_raw_gps_t;


/**
 * @struct data_raw_t
 * @brief Estructura que agrupa todos los datos crudos de los sensores.
 *
 * Contenedor para los datos sin procesar de BMP280, MPU6050 y GPS.
 */
typedef struct {
    data_raw_bmp_t bmp;  ///< Datos crudos del BMP280
    data_raw_mpc_t mpc;  ///< Datos crudos del MPU6050
    data_raw_gps_t gps;  ///< Datos crudos del GPS
} data_raw_t;



/**
 * @struct data_gse_t
 * @brief Estructura de datos para la Estación de Tierra (GSE).
 *
 * Contiene todos los datos procesados y validados de los sensores
 * listos para transmisión a la estación de control terrestre.
 *
 * @see data_all_t
 */
typedef struct { // TODO: Revisar si esta estructura es necesaria o si podemos usar directamente data_all_t para la transmisión a GSE.
    // BMP280
    float presion;          ///< Presión en hPa
    float temperatura_bmp;  ///< Temperatura BMP280 en °C
    float altitud;          ///< Altitud en metros

    // MPU6050
    float accel_x;          ///< Aceleración en eje X en m/s²
    float accel_y;          ///< Aceleración en eje Y en m/s²
    float accel_z;          ///< Aceleración en eje Z en m/s²
    float gyro_x;           ///< Velocidad angular eje X en °/s
    float gyro_y;           ///< Velocidad angular eje Y en °/s
    float gyro_z;           ///< Velocidad angular eje Z en °/s
    float temperatura_mpu;  ///< Temperatura MPU6050 en °C

    // GPS
    double latitud;         ///< Latitud en grados
    double longitud;        ///< Longitud en grados
    uint32_t satelites;     ///< Número de satélites
    bool gps_valido;        ///< Validez del GPS
} data_gse_t;

/**
 * @struct data_all_t
 * @brief Estructura universal de todos los datos del sistema.
 *
 * Contiene la información completa y procesada de todos los sensores:
 * BMP280 (presión y temperatura), MPU6050 (aceleración y giroscopío),
 * y GPS (posicionamiento global).
 *
 * @note Esta estructura se utiliza para almacenar y transmitir todas
 * las mediciones del sistema en una única entidad.
 *
 * @example
 * @code{.c}
 * // Ejemplo de uso
 * data_all_t datos;
 *
 * datos.bmp.presion = 1013.25f;
 * datos.bmp.temperatura_bmp = 24.5f;
 * datos.bmp.altitud = 120.0f;
 *
 * datos.mpc.altitud = 121.0f;
 * datos.mpc.velocidad = 15.2f;
 * datos.mpc.posicion = 42.0f;
 *
 * datos.gps.latitud = -34.6037;
 * datos.gps.longitud = -58.3816;
 * datos.gps.satelites = 8;
 * datos.gps.gps_valido = true;
 * @endcode
 */
typedef struct { // TODO: Completar todos los datos
    // BMP280
    float presion;          ///< Presión en hPa
    float temperatura_bmp;  ///< Temperatura BMP280 en °C
    float altitud;          ///< Altitud en metros

    // MPU6050
    float accel_x;          ///< Aceleración en eje X en m/s²
    float accel_y;          ///< Aceleración en eje Y en m/s²
    float accel_z;          ///< Aceleración en eje Z en m/s²
    float gyro_x;           ///< Velocidad angular eje X en °/s
    float gyro_y;           ///< Velocidad angular eje Y en °/s
    float gyro_z;           ///< Velocidad angular eje Z en °/s
    float temperatura_mpu;  ///< Temperatura MPU6050 en °C

    // GPS
    double latitud;         ///< Latitud en grados
    double longitud;        ///< Longitud en grados
    uint32_t satelites;     ///< Número de satélites
    bool gps_valido;        ///< Validez del GPS
} data_all_t;




#endif //ACEMA_CTLR_DATA_H


