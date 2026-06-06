//
// Created by zhl on 6/6/26.
//

#ifndef ACEMA_CTLR_COMMS_H
#define ACEMA_CTLR_COMMS_H

typedef enum PROTOCOLO {
    ERROR = 0x00,
    MSG = 0x10,
    PLOT = 0x01
} protocolo_enum;

typedef struct paquete_datos {
    void* payload;
    int len;
} pkt_t;

#endif //ACEMA_CTLR_COMMS_H
