//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MGPS_H
#define ACEMA_CTLR_MGPS_H

#include <TinyGPS++.h>
#include <freertos/queue.h>

// Estructura de datos PVT (Position, Velocity, Time) para el lazo de control
struct data_gps_t {
    double latitude;     // Grados
    double longitude;    // Grados
    double altitude;     // Metros sobre el nivel del mar
    double speed;        // m/s
    double course;       // Grados
    float hdop;          // Dilución Horizontal de Precisión (< 2.0 es ideal para lanzamiento)
    float pdop;          // Dilución de precisión (pDOP * 0.01f) menor o igual a 2.0
    uint8_t fix_type;    // 1 = Sin Fix, 2 = Fix 2D, 3 = Fix 3D (Aeronáutico)
    uint32_t satellites; // Cantidad de satélites visibles
    bool gnss_fix_ok;        // Estado de validación (True solo si hay Fix 3D y HDOP aceptable)
    uint32_t last_update;// Timestamp (millis) de la última trama válida procesada
};

class mGPS {
public:
    // Constructor con los parámetros por defecto
    explicit mGPS(int uartNum = 2, int rx = 16, int tx = 17, uint32_t baud = 9600);
    ~mGPS();

    // API Pública de Vuelo
    bool init();                        // Retorna true SOLO si la configuración aeronáutica fue confirmada
    data_gps_t get_gps_raw_data() const; // Lectura instantánea no bloqueante (cero deadlocks)

private:
    // Parámetros de hardware
    int _uartNum;
    int _rxPin;
    int _txPin;
    uint32_t _baud;

    TinyGPSPlus _gps;

    QueueHandle_t _gpsQueue; // usamos freertos queues para no usar mutexes

    static mGPS _instance;
};

#endif //ACEMA_CTLR_MGPS_H