//
// Created by zhl on 6/11/26.
// Refactorizado para misión ACEMA (Vuelo real con redundancia)
//

#ifndef ACEMA_CTLR_MDE_COHETE_H
#define ACEMA_CTLR_MDE_COHETE_H

#include <cstdint>
#include <cmath>
#include "data.h"
#include "../funciones_de_estado.h"
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
    ST_INIT = 0,
    ST_WARMUP_MPU,             // Calentamiento térmico obligatorio de 5 min
    ST_BUSCANDO_CONEXION_GSE,  // Intento de enlace GSE (No bloqueante, con timeout)
    ST_ESPERA_GPS_FIX,         // Esperando 3D Fix
    ST_ESPERA_DESPEGUE,        // En rampa. Ignición externa. Esperando trigger cinemático
    ST_PROPULSION,             // Impulso detectado (>= 2g x 150ms + 4m)
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

/** @enum cod_error_t
 * @brief Códigos de error instantáneos y de diagnóstico
 */
typedef enum {
    ERR_NINGUNO = 0,
    ERR_TIMEOUT_CONEXION_GSE,    // Advertencia: Volando sin telemetría GSE
    ERR_GPS_TIMEOUT,             // Por si queremos forzar el lanzamiento sin GPS (override)
    ERR_MPU_CALIBRACION_FALLIDA, // El sensor no logró estabilizar offsets
    ERR_DESPEGUE_FALSO_ZARANDEO, // Se detectó un pico de Gs pero sin delta de altura
    ERR_TRAYECTORIA_NO_VERTICAL, // El vector de actitud se inclinó peligrosamente
    ERR_DROGUE_DESGARRO,         // Aceleración anómala detectada durante los 3s de drogue
    ERR_FRENADO_AERO_ATASCADO,   // Actuador de frenado aerodinámico no responde
    ERR_SD_COMPUTADORA_MUERTA    // Falló SD principal, derivando a UART_Camara
} cod_error_t;


typedef struct {
    estado_vuelo_t estado;
    cod_error_t error;

    // Flags de configuración de misión
    bool vuelo_en_silencio_radio; ///< True si GSE dio timeout y lanzamos igual
    bool usar_uart_camara_como_sd; ///< True si la SD principal falló

    // Tracking de integradores temporales (Filtros anti-ruido)
    uint64_t timestamp_entrada_estado;
    uint64_t timestamp_inicio_pico_g;  ///< Mide los 150ms continuos de >= 2G
    uint64_t timestamp_apertura_drogue;

    struct {
        // Datos Barométricos puros (BMP280)
        float cota_suelo_rampa;     ///< Altura de tara inicial (~3m)
        float altura_actual_filtrada;
        float altura_max_historica;
        float temperatura_ambiente; ///< Usada exclusivamente para densidad
        float densidad_aire;

        // Datos Inerciales transformados al sistema Suelo (MPU6050 + Filtro)
        Vector3D<float> acel_world; ///< Ya restada la gravedad (-1g en Y)
        Vector3D<float> vel_world;
        Vector3D<float> pos_world;

        // Cuaternión de transformación de coordenadas (Cuerpo -> Suelo)
        float cuaternion_actitud[4];
        bool gps_3d_fix_obtenido;
        int satelites_visibles;
        float gps_hdop;
    } contexto_fisico;

} mde_data_t;

// DEFINICIÓN DE LA INSTANCIA GLOBAL DE VUELO
mde_data_t COHETE = {
    .estado = ST_INIT,
    .error = ERR_NINGUNO,
    .vuelo_en_silencio_radio = false,
    .usar_uart_camara_como_sd = false,
    .timestamp_entrada_estado = 0,
    .timestamp_inicio_pico_g = 0,
    .timestamp_apertura_drogue = 0,
    .contexto_fisico = {
        .cota_suelo_rampa = 0.0f,
        .altura_actual_filtrada = 0.0f,
        .altura_max_historica = 0.0f,
        .temperatura_ambiente = 15.0f, // Valor por defecto sensato
        .densidad_aire = 1.225f,       // Densidad a nivel del mar estándar
        .acel_world = {0.0f, 0.0f, 0.0f},
        .vel_world = {0.0f, 0.0f, 0.0f},
        .pos_world = {0.0f, 0.0f, 0.0f},
        .cuaternion_actitud = {1.0f, 0.0f, 0.0f, 0.0f} // Identidad
    }
};

/** @brief Tabla de punteros a función estrictamente mapeada a estado_vuelo_t */
constexpr f_st_t MDE_COHETE[] = {
    f_st_inicializar_sistema,
    f_st_ejecutar_warmup_mpu,
    f_st_buscar_enlace_gse_timeout,
    f_st_esperar_gps_fix,
    f_st_esperar_impulso,
    f_st_gestionar_ascenso_propulsado,
    f_st_gestionar_vuelo_balistico_y_freno,
    f_st_disparar_apogeo_y_detener_cam,
    f_st_evaluar_supervivencia_drogue,
    f_st_descenso_controlado_drogue,
    f_st_desplegar_principal_emergencia,
    f_st_ejecutar_panico_flash_dump,
    f_st_transmitir_baliza_aterrizaje,
    f_st_procesar_falla_de_sistema
};

void mde_cohete_actualizar(data_all_t* datos_sensores);
void manejar_error(data_all_t* datos_sensores);

#endif //ACEMA_CTLR_MDE_COHETE_H