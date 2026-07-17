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
        ST_DROGUE_DESPLEGADO,      // Derivada de altura nula. Disparo Drogue + Corte cámara. Luego de 3 segundos post-drogue testear salud
        ST_PCAIDAS_PPAL_DESPLEGADO,
        ST_CAIDA_CATASTROFICA,     // Falla total de retención. Pánico -> Volcado a Flash
        ST_ATERRIZAJE,             // Reposo en suelo. Emisión de coordenadas GPS
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
        estado_cohete_t _estado;
        error_cohete_t _error; // contiene el último error que se dio
        bool _entrando_estado;
        // bool es_estado_salida;

        bool drogue_disparado;

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


        // Tracking de integradores temporales (Filtros anti-ruido)
        uint64_t timestamp_micros_entrada_estado; // se actualiza cada vez que entramos a un nuevo estado de la mde
        uint64_t timestamp_millis_inicio_pico_g;  ///< Mide los 150ms continuos de >= 2G
        uint64_t timestamp_micros_apertura_drogue;

        struct {
            float altura_m_max_historica;
            uint32_t masa_g_cohete;
            uint32_t masa_g_combustible;
            float altitud_m_pad; ///< Altura de tara inicial (~3m) --> Se configura a traves de comandos GSE "TARA_INICIAL" --> guardamos el valor de ese instante de datos_sensores->altitud_filtrada_m
            float altitud_m_relativa_al_pad;
        } ctx_fisico;

    } system_data_t;

    extern system_data_t SYSTEM; // SOLAMENTE DEBE SER MODIFICADA POR LA MDE DEL COHETE. LOS DEMÁS PROCESOS SOLO DEBERÍAN LEERLA, PERO NO DEBEN MODIFICARLA. // TODO: FORZAR SOLO LECTURA PARA OBJETOS EXTERNOS A LA MDE.

}






#endif //ACEMA_CTLR_INCLUDE_H
