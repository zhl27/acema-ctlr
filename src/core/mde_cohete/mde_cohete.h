//
// Created by zhl on 6/11/26.
// Refactorizado para misión ACEMA (Vuelo real con redundancia)
//

#ifndef ACEMA_CTLR_MDE_COHETE_H
#define ACEMA_CTLR_MDE_COHETE_H

#include <cstdint>
#include <cmath>
#include "data.h"
#include "funciones_de_estado.h"
#include "Vector3D.h"

/* =========================================================================
 * CONSULTAS DE DISEÑO / INCERTIDUMBRES (Para revisar con el equipo)
 * =========================================================================
 *
 * 1. SOBRE EL EJE VERTICAL:
 * En tu pseudocódigo usaste 'ay' y 'vy'. Estoy asumiendo que tu software
 * trata el EJE Y como el vector normal al suelo (típico de motores gráficos).
 * En aeronáutica el estándar inercial suele ser el eje Z (Z-Down o Z-Up).
 * Si su vertical física calibrada es Z, hay que cambiar los '.y' por '.z'.
 *
 * 2. SOBRE EL SHOCK ESTRUCTURAL DEL PARACAÍDAS PRINCIPAL:
 * Si el drogue falla y venimos a -35 m/s (o peor), abrir el principal de
 * golpe genera un "Opening Shock" brutal. ¿La cuerda de retención (shock cord)
 * y los cáncamos de la bahía de recuperación soportan los Newtons de
 * desacelerar esa masa a esa velocidad, o corremos riesgo de arrancar la bahía?
 *
 * 3. SOBRE LA SD DE LA CÁMARA POR UART:
 * Enviar datos por TX (Compu) a RX (Cámara) asume que la cámara tiene un
 * microcontrolador propio ejecutando un firmware que sabe agarrar lo que entra
 * por su UART y anexarlo a un archivo .txt en su SD. Una tarjeta SD física
 * habla protocolo SPI o SDIO, no entiende UART nativo. Verificar este puente.
 *
 * 4. SOBRE EL TIMEOUT DE CONEXIÓN GSE:
 * Definí un flag 'vuelo_en_silencio_radio'. Si salimos a volar sin enlace,
 * ¿queremos que el cohete intente reconectar continuamente en segundo plano
 * durante el ascenso, o apagamos el módulo de radio para ahorrar batería?
 * ========================================================================= */

/** @enum estado_vuelo_t
 * @brief Máquina de estados secuencial de vuelo ACEMA
 */
typedef enum {
    ST_INIT = 0, //
    // ST_WARMUP_MPU,             // Calentamiento térmico obligatorio de 5 min
    ST_ESPERA_CONEXION_GSE,    // Intento de enlace GSE (No bloqueante, con timeout) --> queremos volar aún sin conexion con GSE
    ST_ESPERA_GPS_PRECISO,         // Esperando 3D Fix
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
} estado_vuelo_t;

// enum EstadoVuelo : uint8_t {
//     IDLE_PAD = 0,
//     IMPULSO_ASCENSO = 1,
//     VUELO_BALISTICO = 2,
//     APOGEO_DETECTADO = 3,
//     DESCENSO_DROGUE = 4,
//     DESCENSO_PRINCIPAL = 5,
//     ATERRIZADO = 6
// };

/** @enum cod_error_t
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
} cod_error_t;


typedef struct {
    estado_vuelo_t estado;
    cod_error_t error;
    bool entrando_estado;
    // bool es_estado_salida;

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

// DEFINICIÓN DE LA INSTANCIA GLOBAL DE VUELO
extern system_data_t COHETE;

bool es_entrada_a_estado();
void transicionar_hacia(estado_vuelo_t nuevo_estado);

/** @brief Tabla de punteros a función estrictamente mapeada a estado_vuelo_t */
extern const f_st_t MDE_COHETE[];

void mde_cohete_actualizar(data_all_t* datos_sensores);
void transicion_error(const cod_error_t &error, data_all_t* datos_sensores);


#endif //ACEMA_CTLR_MDE_COHETE_H