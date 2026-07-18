#ifndef ACEMA_CTLR_MGPS_H
#define ACEMA_CTLR_MGPS_H

#include <TinyGPS++.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <HardwareSerial.h>

#include "data.h"


// SOLO APTO PARA MEDICIONES EN ENTORNOS ESTABLES --> DURANTE EL VUELO PUEDE SER INESTABLE --> NOS IMPORTA PODER RECUPERAR EL COHETE --> CUANDO ATERRICE, LA SEÑAL GPS SE ESTABILIZA.
class mGPS {
public:
    explicit mGPS(int uartNum = 2, int rx = 16, int tx = 17, uint32_t baud = 9600);
    ~mGPS();

    static void _gpsTask(void * pvParameters);

    bool init();
    data_gps_t get_gps_raw_data() const;
    bool test_connection_passed() const;

private:
    int _uartNum;
    int _rxPin;
    int _txPin;
    uint32_t _baud;
    uint32_t _timestamp_init;


    TinyGPSPlus _gps;
    QueueHandle_t _gpsQueue;
    HardwareSerial* _serial;
    TaskHandle_t _taskHandle;
};

#endif //ACEMA_CTLR_MGPS_H