//
// Created by lucaz on 6/7/2026.
//

#ifndef ACEMA_CTLR_INCLUDE_H
#define ACEMA_CTLR_INCLUDE_H

#include <cstdint>
#include "Vector3D.h"



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
        ST_APOGEO,                 // Derivada de altura nula. Disparo Drogue + Corte cámara
        ST_DESCENSO_EVALUACION,    // Ventana de 3 segundos post-drogue para testear salud
        ST_DESCENSO_NOMINAL,       // Drogue OK. Esperando cota de 250m para Principal
        ST_DESCENSO_EMERGENCIA,    // Drogue fallido (vy <= -35 m/s). Disparo Principal de auxilio
        ST_CAIDA_CATASTROFICA,     // Falla total de retención. Pánico -> Volcado a Flash
        ST_ATERRIZAJE,             // Reposo en suelo. Emisión de coordenadas GPS
        ST_ERROR,                  // Estado de captura de excepciones
        ST_NULL
    } estado_cohete_t;

    // enum EstadoVuelo : uint8_t {
    //     IDLE_PAD = 0,
    //     IMPULSO_ASCENSO = 1,
    //     VUELO_BALISTICO = 2,
    //     APOGEO_DETECTADO = 3,
    //     DESCENSO_DROGUE = 4,
    //     DESCENSO_PRINCIPAL = 5,
    //     ATERRIZADO = 6
    // };

    /** @enum error_cohete_t
     * @brief Códigos de error instantáneos y de diagnóstico
     */
    typedef enum {
        ERR_NINGUNO = 0,
        ERR_TIMEOUT_CONEXION_GSE,    // Advertencia: Volando sin telemetría GSE
        ERR_GPS_TIMEOUT,             // Por si queremos forzar el lanzamiento sin GPS (override)
        ERR_MPU_CALIBRACION_FALLIDA, // El sensor no logró estabilizar offsets
        ERR_DESPEGUE_FALSO_ZARANDEO, // Se detectó un pico de Gs pero sin delta de altura
        ERR_DESPEGUE_PROHIBIDO,      // Se realizo despegue a pesar de no estar en condiciones
        ERR_TRAYECTORIA_NO_VERTICAL, // El vector de actitud se inclinó peligrosamente
        ERR_DROGUE_DESGARRO,         // Aceleración anómala detectada durante los 3s de drogue
        ERR_FRENADO_AERO_ATASCADO,   // Actuador de frenado aerodinámico no responde
        ERR_ESTADO_INVALIDO,         // cuando un estado_vuelo_t es mayor que ST_NULL
        ERR_DESCONOCIDO
    } error_cohete_t;


    typedef struct {
        estado_cohete_t estado;
        error_cohete_t error; // contiene el ultimo error que se dio
        bool entrando_estado;
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

        float masa_cohete_kg;
        float altitud_cero_pad_m;

        // Tracking de integradores temporales (Filtros anti-ruido)
        uint64_t timestamp_micros_entrada_estado;
        uint64_t timestamp_micros_inicio_pico_g;  ///< Mide los 150ms continuos de >= 2G
        uint64_t timestamp_micros_apertura_drogue;

        struct {
            // Datos Barométricos puros (BMP280)
            float cota_suelo_rampa;         ///< Altura de tara inicial (~3m)
            float altura_actual;
            float altura_max_historica;

            // Datos Inerciales transformados al sistema Suelo (MPU6050 + Filtro)
            Vector3D<float> acel_global;    ///< Ya restada la gravedad (-1g en Y)
            Vector3D<float> vel_global;
            Vector3D<float> pos_global;

            // Cuaternión de transformación de coordenadas (Cuerpo -> Suelo)
            float cuaternion_actitud[4]; // esto sirve para conocer posicion respecto al punto de origen a todo momento.  // TODO: chequear
            // bool gps_3d_fix_obtenido;
            // int satelites_visibles;
            // float gps_hdop;
        } contexto_fisico;

    } system_data_t;

    extern system_data_t SYSTEM;

}






#endif //ACEMA_CTLR_INCLUDE_H
