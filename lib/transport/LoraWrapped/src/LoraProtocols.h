#ifndef LORA_PROTOCOLS_H
#define LORA_PROTOCOLS_H
#include <cstdint>

/**
 * @enum lora_protocol
 * @brief Define los protocolos de comunicación entre los módulos de comunicación
 */
enum lora_protocol: uint8_t {
    // CPU --> GSE
    C_PLOT    = 0X01,
    C_MGS     = 0X02,
    C_ERR     = 0X03,
    PING    = 0X04,
    C_ACK     = 0X05,

    // GSE --> CPU
    G_START   = 0X10,
    G_END     = 0X20,
    PONG    = 0X30,
    G_CMD     = 0X40 
};

// Estructura para el Acuse de Recibo (ACK)
struct CmdAck_t {
    uint32_t opCode;
    int8_t status;
    float datoOpcional;
};
#endif /* LORA_PROTOCOLS_H */