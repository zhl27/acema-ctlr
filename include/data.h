//
// Created by zhl on 6/6/26.
//


#ifndef ACEMA_CTLR_DATA_H
#define ACEMA_CTLR_DATA_H

#include <Arduino.h>

#include <cstdint>
#include <cstdio>
#include "freertos/FreeRTOS.h"
 
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
    uint64_t timestamp_micros; // queremos saber a qué tiempo exacto se tomaron estos datos.

    // --- CINEMÁTICA ANGULAR (Velocidades) ---
    float vel_angular_z_deg_s;              // Yaw rate (°/s) --> Exclusivo MPU6050
    float vel_angular_y_deg_s;              // Roll rate (°/s) --> Exclusivo MPU6050
    float vel_angular_x_deg_s;              // Pitch rate (°/s) --> Exclusivo MPU6050

    // --- CINEMÁTICA ANGULAR (Ángulos Absolutos) ---
    float angulo_pitch_deg;                 // Ángulo Pitch filtrado por Kalman (°) --> Exclusivo MPU6050
    float angulo_yaw_deg;                   // Ángulo Yaw filtrado por Kalman (°) --> Exclusivo MPU6050
    float angulo_respecto_z_deg;            // Inclinación total del cohete --> Exclusivo MPU6050

    // --- CINEMÁTICA LINEAL (Eje Y es nuestro eje vertical) ---
    float altitud_filtrada_m_bmp;           // Altura filtrada sobre el nivel del mar --> Exclusivo BMP280 // TODO: Necesitamos tener valor de altitud que salga solamente del bmp280.
    float aceleracion_vertical_m_s2;        // Aceleración lineal absoluta (sin gravedad) --> Exclusivo MPU6050
    float altitud_filtrada_m;               // Altura filtrada sobre el nivel del mar --> BMP280 + MPU6050
    float velocidad_vertical_filtrada_m_s;  // Velocidad vertical real --> BMP280 + MPU6050

    float momentum_kg_m_s;                  // Cantidad de movimiento (P = m * v)
    float temperatura_amb_c;                // Tomada estrictamente del BMP280
    float densidad_aire_kg_m3;              // Calculada por ley de gases ideales --> Exclusivo BMP280

    // --- GPS ---
    bool gps_is_valid;
    uint32_t gps_nro_satelites;
    float gps_hdop;
    float gps_latitud;
    float gps_longitud;

    
    // TODO: MOVER LOS SIGUIENTES CAMPOS HACIA data_gse_t
    ///> DATOS EXTRAS PARA ENVIAR POR GSE

    float angulo_airbrake;

    bool hay_continuidad_pyro_pcaidas_ppal;
    bool hay_continuidad_pyro_pcaidas_drogue;

    uint32_t estado_vuelo;
    uint32_t error_vuelo;

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

    data_all_t datos_filtrados;

    float angulo_airbrake;

    bool hay_continuidad_pyro_pcaidas_ppal;
    bool hay_continuidad_pyro_pcaidas_drogue;

    uint32_t estado_vuelo;
    uint32_t error_vuelo;

} data_gse_t;


#include "../src/core/math/Vector3f.h"

/**
 * @brief Datos primordiales para cargar en caso de reinicio
 */
struct __attribute__((__packed__)) ConfigDatos { // TODO: Esto me inspira la idea de un Process Control Block.

    // Estructuras para los bias de calibración
// Estructuras para los bias de calibración (POD puro)
    struct bias_t{
        math::Vector3f gyro; 
        math::Vector3f accel;
    } bias;

    // Estructura para los alfa de los filtros EMA
    struct {
        float temp, presion, densidad, accelVert;
    }alfaEma;

    int estado_cohete_actual; // es un enum

    // Coeficiente para filtrado
    float altitud_base_agl;         // CRÍTICO: Presión o altitud nivel del suelo
    float altitud_actual_agl;  
    float altura_max_historica_m;
     
    bool mision_activa;             // false = En tierra/Test, true = Vuelo armado/En curso
//    char estado_calibracion[10]; 
};



namespace Cohete {

    static const char *TAG_BASE = "STATE MACHINE";
    // const size_t TAG_BASE_LEN = strlen(TAG_BASE)+1;
    // extern const size_t TAG_MAX_LEN;

    /** @enum estado_cohete_t
     * @brief Máquina de estados secuencial de vuelo ACEMA
     */
    typedef enum {
        ST_INIT = 0, //
        ST_ESPERA_CONEXION_GSE,    // Intento de enlace GSE (No bloqueante, con timeout) --> queremos volar aún sin conexion con GSE
        ST_ESPERA_GPS_PRECISO,     // Esperando 3D Fix
        ST_ESPERA_IGNICION,        // En rampa. Ignición externa. Esperando trigger cinemático
        ST_BOOST,                  // Impulso detectado (>= 2g x 150ms + 4m)
        ST_FASE_BALISTICA,         // Inercia ascendente. Activa rutina de frenado aerodinámico
        ST_DROGUE_DESPLEGADO,      // Derivada de altura nula. Disparo Drogue + Corte cámara. Luego de 3 segundos post-drogue testear salud
        ST_PCAIDAS_PPAL_DESPLEGADO,
        ST_CAIDA_CATASTROFICA,     // Falla total de retención. Pánico -> Volcado a Flash
        ST_ATERRIZADO,             // Reposo en suelo. Emisión de coordenadas GPS
        ST_NULL
    } estado_cohete_t;


    /** @enum error_cohete_t
     * @brief Códigos de error instantáneos y de diagnóstico
     */
    typedef enum {
        ERR_NINGUNO = 0,
        ERR_TIMEOUT_CONEXION_GSE,    // Advertencia: Volando sin telemetría GSE
        ERR_GPS_TIMEOUT,             // Por si queremos forzar el lanzamiento sin GPS (override)
        ERR_MPU_CALIBRACION_FALLIDA, // El sensor no logró estabilizar offsets
        ERR_DESPEGUE_FALSO_ZARANDEO, // Se detectó un pico de Gs pero sin delta de altura
        ERR_DESPEGUE_PROHIBIDO,      // Se realizó despegue a pesar de no estar en condiciones
        ERR_TRAYECTORIA_NO_VERTICAL, // El vector de actitud se inclinó peligrosamente
        ERR_DROGUE_NO_EFECTO,         // Aceleración anómala detectada durante los 3s de drogue
        ERR_FRENADO_AERO_ATASCADO,   // Actuador de frenado aerodinámico no responde
        ERR_ESTADO_INVALIDO,         // cuando un estado_vuelo_t es mayor que ST_NULL
        ERR_DESCONOCIDO
    } error_cohete_t;


    typedef struct {
        estado_cohete_t estado;
        estado_cohete_t estado_anterior;
        error_cohete_t _error; // contiene el último error que se dio
        // bool entrando_estado;
        // bool es_estado_salida;


        struct {
            TaskHandle_t xTaskReadSensorsHandle;
            TaskHandle_t xTaskStateMachineHandle;
            TaskHandle_t xTaskFlashHandle;
            TaskHandle_t xTaskLoraHandle;
            TaskHandle_t xTaskDataFilterHandle;
            struct {
                bool Sensors_a_StateMachine_enabled;
                bool Sensors_a_Flash_enabled;
                bool Sensors_a_Lora_enabled;
            } flujos;
        } procesos;

        struct {
            bool gps_override_skip; // esto se configura con un comando desde la GSE --> // TODO: Cómo manejamos esto sin utilizar los temibles mutexes ?
            // bool en_condiciones_para_vuelo_override_true; // Innecesario, de todas formas, las condiciones_para_vuelo en Falso no va a detener la ignición del cohete.
        } gse;

        // Tracking de integradores temporales
        uint32_t timestamp_millis_entrada_estado; // se actualiza cada vez que entramos a un nuevo estado de la mde
        uint64_t _timestamp_millis_inicio_pico_g;  ///< Mide los 150ms continuos de >= 2G
        uint64_t timestamp_micros_apertura_drogue;

        struct {
            float altura_m_max_historica;
            uint32_t masa_g_cohete;
            uint32_t masa_g_combustible;
            float altitud_m_pad; ///< Altura de tara inicial (~3m) --> Se configura a traves de comandos GSE "TARA_INICIAL" --> guardamos el valor de ese instante de datos_sensores->altitud_filtrada_m
            float altitud_m_relativa_al_pad;
            float gps_ultima_latitud_valida;
            float gps_ultima_longitud_valida;
        } ctx_fisico;

        // PEGAMENTO FEO, NECESITO ACCESO A LA FLASH
        // void* blackBox; 
        struct{
            // bool gse_conectado;
            bool flash_log_borrado;
            bool gps_preciso;
            bool drogue_disparado;
            bool paracaidas_principal_disparado;
            bool emergencia_fatal;
            bool borrar_log;
            bool volcar_ram_a_flash;
        } flags;

    } system_data_t;

    extern system_data_t SYSTEM; // SOLAMENTE DEBE SER MODIFICADA POR LA MDE DEL COHETE. LOS DEMÁS PROCESOS SOLO DEBERÍAN LEERLA, PERO NO DEBEN MODIFICARLA. // TODO: FORZAR SOLO LECTURA PARA OBJETOS EXTERNOS A LA MDE.

}

















#include <Arduino.h>

// ============================================================================
// 1. FUNCIONES ATÓMICAS - DATOS CRUDOS (RAW)
// ============================================================================

inline void plot_mpu_raw(const data_raw_t *data) {
    if (data == nullptr) return;

    Serial.print(">ax_m_s2:");   Serial.println(data->mpu.accel_x_m_s2, 4);
    Serial.print(">ay_m_s2:");   Serial.println(data->mpu.accel_y_m_s2, 4);
    Serial.print(">az_m_s2:");   Serial.println(data->mpu.accel_z_m_s2, 4);
    Serial.print(">gx_rad_s:");  Serial.println(data->mpu.gyro_x_rad_s, 4);
    Serial.print(">gy_rad_s:");  Serial.println(data->mpu.gyro_y_rad_s, 4);
    Serial.print(">gz_rad_s:");  Serial.println(data->mpu.gyro_z_rad_s, 4);
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
// ============================================================================
// FUNCIONES ATÓMICAS REFACTORIZADAS (Anti-Crash Heap)
// ============================================================================

inline void plot_cinematica_filtrada(const data_all_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    
    Serial.print(">alt_agl_m:");    Serial.println(data->altitud_filtrada_m, 2);
    Serial.print(">vel_z_m_s:");    Serial.println(data->velocidad_vertical_filtrada_m_s, 2);
    Serial.print(">accel_z_m_s2:"); Serial.println(data->aceleracion_vertical_m_s2, 2);
#endif
}



inline void plot_actitud_filtrada(const data_all_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    
    Serial.print(">pitch_deg:");  Serial.println(data->angulo_pitch_deg, 2);
    Serial.print(">yaw_deg:");    Serial.println(data->angulo_yaw_deg, 2);
    Serial.print(">incl_z_deg:"); Serial.println(data->angulo_respecto_z_deg, 2);
#endif

}

inline void plot_gps_procesado(const data_all_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    
    Serial.print(">lat:");  Serial.println(data->gps_latitud, 6);
    Serial.print(">lon:");  Serial.println(data->gps_longitud, 6);
    Serial.print(">hdop:"); Serial.println(data->gps_hdop, 2);
    Serial.print(">sats:"); Serial.println(data->gps_nro_satelites);
#endif

}


inline void plot_giroscopio_filtrado(const data_all_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    Serial.printf(">vel_ang_x_deg_s:%f\n>vel_ang_y_deg_s:%f\n>vel_ang_z_deg_s:%f\n",
                  data->vel_angular_x_deg_s,
                  data->vel_angular_y_deg_s,
                  data->vel_angular_z_deg_s);
#endif

}

inline void plot_ambiental_procesado(const data_all_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    Serial.printf(">temp_amb_c:%f\n>densidad_aire:%f\n",
                  data->temperatura_amb_c,
                  data->densidad_aire_kg_m3);
#endif

}


// ============================================================================
// 3. WRAPPERS QUE ENGLOBAN TODO (ALL IN ONE)
// ============================================================================

/**
 * @brief Imprime en una sola línea del plotter todos los sensores en estado CRUDO
 */
inline void plot_all_raw(const data_raw_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    Serial.printf(">presion_hpa:%f\n>temp_c:%f\n>ax_m_s2:%f\n>ay_m_s2:%f\n>az_m_s2:%f\n>gx_rad_s:%f\n>gy_rad_s:%f\n>gz_rad_s:%f\n>lat:%ld\n>lon:%ld\n>sats:%d\n",
                  data->bmp.presion_hpa, data->bmp.temp_deg_c,
                  data->mpu.accel_x_m_s2, data->mpu.accel_y_m_s2, data->mpu.accel_z_m_s2,
                  data->mpu.gyro_x_rad_s, data->mpu.gyro_y_rad_s, data->mpu.gyro_z_rad_s,
                  (long)data->gps.latitude, (long)data->gps.longitude, (int)data->gps.satellites);
#endif
}

/**
 * @brief Imprime en una sola línea del plotter todas las variables PROCESADAS / FILTRADAS
 */
inline void plot_all_processed(const data_all_t *data) {
#ifdef DEBUG_ESP32
    if (data == nullptr) return;
    Serial.printf(">alt_agl:%f\n>vel_z:%f\n>accel_z:%f\n>pitch:%f\n>yaw:%f\n>incl_z:%f\n>vel_ang_x:%f\n>vel_ang_y:%f\n>vel_ang_z:%f\n>temp_amb:%f\n>rho:%f\n>lat:%f\n>lon:%f\n>hdop:%f\n>sats:%d\n",
                  data->altitud_filtrada_m, data->velocidad_vertical_filtrada_m_s, data->aceleracion_vertical_m_s2,
                  data->angulo_pitch_deg, data->angulo_yaw_deg, data->angulo_respecto_z_deg,
                  data->vel_angular_x_deg_s, data->vel_angular_y_deg_s, data->vel_angular_z_deg_s,
                  data->temperatura_amb_c, data->densidad_aire_kg_m3,
                  data->gps_latitud, data->gps_longitud, data->gps_hdop, data->gps_nro_satelites);
#endif
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

/*
 * ESTO NOS PERMITE DEBUGGEAR LOS DATOS
 * USANDO SERIAL PLOTTER DE LA IDE DE ARDUINO.
 */
inline void print_plotter_data_raw(const data_raw_t *data) {
#if defined(PLOT_MPU_RAW) || defined(PLOT_BMP_RAW) || defined(PLOT_GPS_RAW)
    if (data == NULL) return;

#if defined(PLOT_MPU_RAW)
    // Ideal para calibrar offsets, ver ruido y probar el filtro complementario
    Serial.printf(
        // "AccX_raw_m_s2:%f,"
        "AccY_raw_m_s2:%f,"
        // "AccZ_raw_m_s2:%f,"
        // "GyroX_raw_rads:%f,"
        // "GyroY_raw_rads:%f,"
        // "GyroZ_raw_rads:%f"
        "\r\n",
                  // data->mpu.accel_x_m_s2,
                  data->mpu.accel_y_m_s2
                  // data->mpu.accel_z_m_s2
                  // data->mpu.gyro_x_rad_s,
                  // data->mpu.gyro_y_rad_s,
                  // data->mpu.gyro_z_rad_s
    );
#endif

#if defined(PLOT_BMP_RAW)
    // Restamos un offset aproximado (ej. 1000 hPa) a la presión si quieres ver variaciones
    // pequeñas de altura junto con la temperatura sin que se aplasten mutuamente:
    Serial.printf(
        // "Presion_raw_hPa:%f,"
        "Altura_raw_bmp:%f"
        // ",Temp_raw_C:%f"
        "\r\n",
                  // data->bmp.presion_hpa,
                  data->bmp.altitud_snm_m
                  // data->bmp.temp_deg_c
    );
#endif

#if defined(PLOT_GPS_RAW)
    Serial.printf("Lat:%ld,Lon:%ld,Sats:%d\r\n",
                  (long)data->gps.latitude,
                  (long)data->gps.longitude,
                  (int)data->gps.satellites);
#endif
#endif
}


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
    Serial.printf("Velocidad Z:        %.2f m/s\n", data->velocidad_vertical_filtrada_m_s);
    Serial.printf("Aceleración Z:      %.2f m/s^2\n", data->aceleracion_vertical_m_s2);
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
}

inline void print_plotter_data_processed(const data_all_t *data) {
#if defined(PLOT_MPU_PROCESSED) || defined(PLOT_BMP_PROCESSED) || defined(PLOT_GPS_PROCESSED)
    if (data == NULL) return;

#if defined(PLOT_MPU_PROCESSED)
    // Ideal para evaluar el filtro de Kalman/complementario, orientación espacial y cinemática lineal
    Serial.printf(
                // "Pitch_deg:%f,"
                // "Yaw_deg:%f,"
                // "InclZ_deg:%f,"
                // "PitchRate_deg_s:%f,"
                // "RollRate_deg_s:%f,"
                // "YawRate_deg_s:%f,"
                "AccVert_m_s2:%f"
                "\r\n",
        // data->angulo_pitch_deg,
        // data->angulo_yaw_deg,
        // data->angulo_respecto_z_deg,
        // data->vel_angular_x_deg_s,
        // data->vel_angular_y_deg_s,
        // data->vel_angular_z_deg_s,
        data->aceleracion_vertical_m_s2
    );
#endif

#if defined(PLOT_BMP_PROCESSED) && defined(PLOT_MPU_PROCESSED)
    Serial.printf(
        "VelVert_m_s:%f,"
               "AltFusion_m:%f"
               "\r\n",
        data->velocidad_vertical_filtrada_m_s,
        data->altitud_filtrada_m);
#endif

#if defined(PLOT_BMP_PROCESSED)
    // Ideal para comparar la altitud pura del BMP280 vs. la altitud fusionada (BMP + MPU),
    // ver la velocidad vertical, temperatura ambiental y la densidad del aire calculada
    Serial.printf(
        "AltBMP_m:%f,"
                // "Temp_C:%f,"
                // "Densidad_kg_m3:%f,"
                // "Momentum_kg_m_s:%f"
                "\r\n",
        data->altitud_filtrada_m_bmp
          // data->temperatura_amb_c
          // data->densidad_aire_kg_m3
          // ,data->momentum_kg_m_s
          );
#endif

#if defined(PLOT_GPS_PROCESSED)
    // Para visualización de la posición georreferenciada procesada y métricas de calidad de señal
    Serial.printf("Valid:%d,Lat:%f,Lon:%f,Sats:%u,HDOP:%f\r\n",
                  data->gps_is_valid ? 1 : 0,
                  data->gps_latitud,
                  data->gps_longitud,
                  data->gps_nro_satelites,
                  data->gps_hdop);
#endif
#endif
}


// Estructura del payload que viajará por la cola RTOS
struct CommandPayload {
    uint32_t opCode;
    float value;
};

#endif //ACEMA_CTLR_DATA_H



