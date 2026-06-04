//
// Created by zhl on 6/2/26.
//

#ifndef ACEMA_CTLR_GLOBALS_H
#define ACEMA_CTLR_GLOBALS_H

#define UMBRAL_LANZAMIENTO 100 // TODO: falta implementar valor real
#define SERIAL_BAUDRATE 115200

#define configSUPPORT_STATIC_ALLOCATION 1

#define BUZZER_PIN 25
#define WIRE_SDA 21
#define WIRE_SCL 22
#define MPU_ADDR 0x69
#define BMP280_ADDR 0x77


typedef enum PROTOCOLO {
    ERROR = 0x00,
    MSG = 0x10,
    PLOT = 0x01
} protocolo_enum;

typedef struct paquete_datos {
    void* payload;
    int len;
} pkt_t;


#endif //ACEMA_CTLR_GLOBALS_H
