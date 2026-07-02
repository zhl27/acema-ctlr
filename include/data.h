//
// Created by zhl on 6/6/26.
//


#ifndef ACEMA_CTLR_DATA_H
#define ACEMA_CTLR_DATA_H

#include <cstdint>
#include "UbxProtocols.h"

/**
 * @struct data_raw_mpu_t
 * @brief Estructura de datos crudos de la MPU6050.
 *
 * Contiene los datos sin procesar del acelerómetro y giroscopio.
 */
typedef struct {
    int16_t accel_x; // Aceleración X (Raw)
    int16_t accel_y; // Aceleración Y (Raw)
    int16_t accel_z; // Aceleración Z (Raw)
    int16_t temp;    // Temperatura (Raw)
    int16_t gyro_x;  // Velocidad angular X (Raw)
    int16_t gyro_y;  // Velocidad angular Y (Raw)
    int16_t gyro_z;  // Velocidad angular Z (Raw)
} data_raw_mpu_t;

/**
 * @struct data_raw_bmp_t
 * @brief Estructura de datos crudos del BMP280.
 *
 * Contiene los datos sin procesar del sensor barométrico.
 */
typedef struct {
    int32_t presion; ///< Presión cruda (valor de 20 bits 'up') [5]
    int32_t temp; ///< Temperatura cruda (valor de 20 bits 'ut') [6]
} data_raw_bmp_t;

// /**
//  * @struct data_raw_gps_t
//  * @brief Estructura de datos crudos del GPS.
//  *
//  * Contiene los datos sin procesar del receptor GPS.
//  *
//  * Posición: Coordenadas de latitud y longitud con una precisión horizontal autónoma de aproximadamente 2.5 metros (usando GPS) o 4.0 metros (usando GLONASS)
//  * Velocidad: Con una precisión de 0.1 m/s.
//  * Tiempo: Entrega una referencia temporal precisa, incluyendo una señal de pulso de tiempo (TIMEPULSE) configurable con una precisión de nanosegundos.
//  * Trayectoria (Heading): Dirección del movimiento con una precisión de 0.5 grados.
//  *
//  */
// typedef struct {
//     // --- Posicionamiento ---
//     double latitud;           ///> Coordenadas en grados (NMEA ASCII o UBX Binario) [1]
//     double longitud;          ///> Coordenadas en grados (NMEA ASCII o UBX Binario) [1]
//     float altitud;            ///> Altitud en metros (Limite operativo: 50,000 m) [2, 3]
//     float precision_horizontal;        ///> Precision horizontal (Aprox. 2.5m en GPS / 4.0m GLONASS) [2, 3]
//
//     // --- Movimiento ---
//     float velocidad;          ///> Velocidad en m/s (Precision de 0.1 m/s) [2, 3]
//     float heading;            ///> Direccion del movimiento en grados (Precision de 0.5 grados) [2, 3]
//     float dinamica_max;       ///> Aceleracion soportada (Hasta 4g) [2, 3]
//
//     // --- Tiempo y Sincronizacion ---
//     uint32_t tiempo_utc;      ///> Referencia temporal sincronizada [2]
//     uint32_t freq_timepulse;  ///> Frecuencia configurable (0.25 Hz a 10 MHz) [2, 4]
//     uint32_t precision_pulso; ///> Precision de la señal de tiempo en nanosegundos (30ns a 100ns) [2, 3]
//
//     // --- Estado del Sistema ---
//     int nro_satelites;        ///> Numero de satelites (de un motor de 56 canales) [5, 6]
//     bool tiene_fix;           ///> Estado de posicionamiento (TTFF de 1s en Hot Start) [2, 3] --> TIFF es Time-To-First-Fix --> El dato "tiene_fix" indica si el módulo ha logrado sincronizarse con los satélites necesarios para calcular una posición geográfica válida
//     char sistema_activo;      ///> GPS, GLONASS o Galileo (via firmware) [7, 8]
// } data_raw_gps_t;


/**
 * @struct data_raw_t
 * @brief Flujo crudo. Estructura que agrupa todos los datos crudos de los sensores.
 *
 * Contenedor para los datos sin procesar de BMP280, MPU6050 y GPS.
 */
typedef struct {
    data_raw_bmp_t bmp;  ///< Datos crudos del BMP280
    data_raw_mpu_t mpc;  ///< Datos crudos del MPU6050
    nav_pvt_t gps;       ///< Datos crudos del GPS
    uint64_t elapsed_time;  ///< Marca de tiempo de la lectura de los datos
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
typedef struct {
    int16_t posicion_relativa;
    int16_t velocidad;
    int16_t momentum; // inercia
    //estadoVuelo_t vuelo_estado_actual; // se va a ver como un integer

    double latitud;           ///> Coordenadas en grados (NMEA ASCII o UBX Binario) [1]
    double longitud;          ///> Coordenadas en grados (NMEA ASCII o UBX Binario) [1]
    float altitud;            ///> Altitud en metros (Limite operativo: 50,000 m) [2, 3]

    int nro_satelites;        ///> Numero de satelites (de un motor de 56 canales) [5, 6]
    bool tiene_fix;           ///> Estado de posicionamiento (TTFF de 1s en Hot Start) [2, 3] --> TIFF es Time-To-First-Fix --> El dato "tiene_fix" indica si el módulo ha logrado sincronizarse con los satélites necesarios para calcular una posición geográfica válida
    char sistema_activo;      ///> GPS, GLONASS o Galileo (via firmware) [7, 8]
    // TODO: hay que considerar los datos procesados que se infieren de los crudos
    // capaz alguno de los datos crudos no se envía a la GSE, o se envía solo un subconjunto de ellos, o se envían datos procesados derivados de los crudos.
    // Eso depende del diseño de la telemetría y de las necesidades de la GSE.
    uint64_t timestamp;  ///< Marca de tiempo de la lectura de los datos
} data_gse_t;

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
    // data_raw_t data_raw;           // Flujo crudo original // TODO: Ver si hace falta.

    // --- CINEMÁTICA LINEAL (Eje Z absoluto calibrado al cielo) ---
    float altura_m;                   // Altura filtrada sobre el suelo
    float velocidad_z_m_s;            // Velocidad vertical real
    float aceleracion_z_m_s2;         // Aceleración lineal absoluta (sin gravedad)

    // --- DINÁMICA ---
    float momentum_kg_m_s;            // Cantidad de movimiento (P = m * v)

    // --- CINEMÁTICA ANGULAR ---
    float vel_angular_x;              // Pitch rate (deg/s o rad/s)
    float vel_angular_y;              // Roll rate
    float vel_angular_z;              // Yaw rate
    float vel_rotacional_rpm;         // Magnitud escalar del spin centrífugo

    // --- ORIENTACIÓN ESPACIAL (Filtro Complementario) ---
    float pitch_deg;
    float roll_deg;

    // --- AMBIENTALES PROCESADOS ---
    float temperatura_amb_c;          // Tomada estrictamente del BMP280
    float densidad_aire_kg_m3;        // Calculada por ley de gases ideales

    // --- TELEMETRÍA EMPAQUETADA (Tus requerimientos originales) ---
    int16_t posicion_relativa;        // Altura casteada para ahorrar ancho de banda LoRa
    int16_t velocidad;                // Velocidad vertical casteada
    int16_t momentum;                 // Momentum casteado

} data_all_t; ///< Todos los datos originados del ambiente a través de los sensores que YA ESTÁN SANITIZADOS Y FILTRADOS!


#endif //ACEMA_CTLR_DATA_H



#include <cstdint>
