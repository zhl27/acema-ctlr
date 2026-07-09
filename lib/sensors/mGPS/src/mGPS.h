//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MGPS_H
#define ACEMA_CTLR_MGPS_H

#include <Arduino.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "UbxDispatcher.h"
#include "UbxConfigurator.h"

class mGPS {
public:
    // Constructor con los parámetros por defecto solicitados
    mGPS(int uartNum = 2, int rx = 16, int tx = 17, uint32_t baud = 9600);
    ~mGPS();

    // Métodos principales
    void init();
    void update() const;

    // Getters
    uint32_t getSatellites() const;
    double getLatitude() const;
    double getLongitude() const;
    bool is3dFixed() const;
    nav_pvt_t get_gps_raw_data() const;

private:
    // Parámetros de hardware
    int _uartNum;
    int _rxPin;
    int _txPin;
    uint32_t _baud;

    // Almacenamiento de datos
    nav_pvt_t _pvt_data_rx; // Buffer crudo para el Dispatcher
    nav_pvt_t _pvt_data;    // Buffer seguro para el usuario (Getters)

    // Sincronización
    SemaphoreHandle_t _ackSemaphore;
    SemaphoreHandle_t _dataMutex; // Para evitar lectura/escritura concurrente de datos

    // Instancias de u-blox
    UbxDispatcher* _dispatcher;
    UbxConfigurator* _configurator;

    // Tablas de enrutamiento
    UbxRegMsg_t _regPvt;
    UbxRegMsg_t _regAck;
    const UbxRegMsg_t* _tablaRegistros[2];

    // --- MANEJO DE CALLBACKS (Puente C a C++) ---
    static mGPS* _instance;

    static void _onPvtReceivedStatic(void* data);
    static void _onAckReceivedStatic(void* data);
    static void _uartTxStatic(const uint8_t* data, size_t len);
    static bool _waitAckStatic(uint8_t cls, uint8_t id, uint32_t timeoutMs);

    void _onPvtReceived(void* data);
    void _onAckReceived(void* data);
    void _uartTx(const uint8_t* data, size_t len) const;
    bool _waitAck(uint8_t cls, uint8_t id, uint32_t timeoutMs);
};




#endif //ACEMA_CTLR_MGPS_H
