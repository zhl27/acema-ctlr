//
// Created by lucaz on 23/7/2026.
//

#ifndef ACEMA_CTLR_MOCKBMP280_H
#define ACEMA_CTLR_MOCKBMP280_H


#include <cstdint>
#include <cmath>

// =====================================================================
// 1. CONSTANTES Y ENUMERACIONES SIMULADAS (Adafruit_BMP280 / Estándar)
// =====================================================================

#ifndef BMP280_CHIPID
#define BMP280_CHIPID 0x58 // Chip ID por defecto según el datasheet de Bosch
#endif

// Modos de operación del sensor
enum bmp280_mode_t {
    MODE_SLEEP  = 0x00,
    MODE_FORCED = 0x01,
    MODE_NORMAL = 0x03
};

// Sobremuestreo (Oversampling) para Temperatura y Presión
enum bmp280_sampling_t {
    SAMPLING_NONE = 0x00,
    SAMPLING_X1   = 0x01,
    SAMPLING_X2   = 0x02,
    SAMPLING_X4   = 0x03,
    SAMPLING_X8   = 0x04,
    SAMPLING_X16  = 0x05
};

// Configuración del filtro IIR interno
enum bmp280_filter_t {
    FILTER_OFF = 0x00,
    FILTER_X2  = 0x01,
    FILTER_X4  = 0x02,
    FILTER_X8  = 0x03,
    FILTER_X16 = 0x04
};

// Tiempo de espera entre lecturas en Modo Normal (Standby)
enum bmp280_standby_t {
    STANDBY_MS_0_5  = 0x00,
    STANDBY_MS_1    = 0x00, // Alias utilizado en alta velocidad para el cohete
    STANDBY_MS_62_5 = 0x01,
    STANDBY_MS_125  = 0x02,
    STANDBY_MS_250  = 0x03,
    STANDBY_MS_500  = 0x04,
    STANDBY_MS_1000 = 0x05,
    STANDBY_MS_2000 = 0x06,
    STANDBY_MS_4000 = 0x07
};

// =====================================================================
// 2. CLASE MOCK BMP280
// =====================================================================

class mockBMP280 {
public:
    mockBMP280() {
        // Condiciones atmosféricas estándar por defecto (Nivel del mar, ISA)
        mock_temperature_c = 25.0f;     // 25 °C (temperatura ambiente limpia)
        mock_pressure_pa   = 101325.0f; // 1013.25 hPa expresados en Pascales
        mock_altitude_m    = 0.0f;      // 0 metros s.n.m.
    }

    // --- Métodos de Inicialización ---

    bool begin(uint8_t addr = 0x77, uint8_t chipid = BMP280_CHIPID) {
        last_addr   = addr;
        last_chipid = chipid;
        is_initialized = true;
        return true; // Simula una inicialización exitosa en el bus I2C/SPI
    }

    // Alias de inicialización presente en tus firmas de ejemplo
    bool init(uint8_t addr = 0x77, uint8_t chipid = BMP280_CHIPID) {
        return begin(addr, chipid);
    }

    // --- Configuración del Sensor ---

    void setSampling(bmp280_mode_t mode          = MODE_NORMAL,
                     bmp280_sampling_t temp_os   = SAMPLING_X2,
                     bmp280_sampling_t press_os  = SAMPLING_X8,
                     bmp280_filter_t filter      = FILTER_OFF,
                     bmp280_standby_t standby    = STANDBY_MS_1) {
        config_mode     = mode;
        config_temp_os  = temp_os;
        config_press_os = press_os;
        config_filter   = filter;
        config_standby  = standby;
    }

    // --- Lecturas Telemetricas ---

    float readTemperature() {
        if (!is_initialized) return 0.0f;
        return mock_temperature_c;
    }

    float readPressure() {
        if (!is_initialized) return 0.0f;
        return mock_pressure_pa;
    }

    float readAltitude(float seaLevelhPa = 1013.25f) {
        if (!is_initialized) return 0.0f;

        // Si se activó la simulación barométrica dinámica, calcula la altitud
        // usando la fórmula barométrica estándar a partir de la presión actual:
        if (use_calculated_altitude) {
            float pressure_hpa = mock_pressure_pa / 100.0f;
            return 44330.0f * (1.0f - std::pow(pressure_hpa / seaLevelhPa, 0.1903f));
        }

        // De lo contrario, devuelve directamente la altitud sintética inyectada
        return mock_altitude_m;
    }

    // =================================================================
    // 3. MÉTODOS DE INYECCIÓN PARA PRUEBAS UNITARIAS (TESTING)
    // =================================================================

    /**
     * @brief Permite inyectar condiciones atmosféricas y de vuelo sintéticas.
     * Es ideal para simular el apogeo, el descenso bajo paracaídas o la
     * calibración inicial en la rampa de lanzamiento.
     */
    void setMockData(float temperature_c, float pressure_pa, float altitude_m = 0.0f, bool force_altitude = true) {
        mock_temperature_c      = temperature_c;
        mock_pressure_pa        = pressure_pa;
        mock_altitude_m         = altitude_m;
        use_calculated_altitude = !force_altitude;
    }

    /**
     * @brief Atajo rápido para simular únicamente la variación de altitud
     * durante las pruebas del algoritmo de despliegue de paracaídas.
     */
    void setMockAltitude(float altitude_m) {
        mock_altitude_m = altitude_m;
        use_calculated_altitude = false;
    }

    // Variables públicas de inspección para aserciones (asserts) en tests
    bool is_initialized = false;
    uint8_t last_addr   = 0x00;
    uint8_t last_chipid = 0x00;

    bmp280_mode_t     config_mode;
    bmp280_sampling_t config_temp_os;
    bmp280_sampling_t config_press_os;
    bmp280_filter_t   config_filter;
    bmp280_standby_t  config_standby;

private:
    float mock_temperature_c;
    float mock_pressure_pa;
    float mock_altitude_m;
    bool  use_calculated_altitude = false;
};


#endif //ACEMA_CTLR_MOCKBMP280_H
