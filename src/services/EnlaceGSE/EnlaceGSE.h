#ifndef ENLACE_GSE_H
#define ENLACE_GSE_H

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include <cstring>

#include "data.h"          
#include "LoraProtocols.h"   



// El "Sobre" genérico
struct TxEnvelope_t {
    lora_protocol tipo; 
    union {
        data_all_t telemetria;
        char texto[128]; // De la configuración del lora
        CmdAck_t ack;
    } payload;
};



class EnlaceGSE {
private:
    // Puntero interno a la cola
    static RingbufHandle_t _ringbuf;
    
    // Método privado centralizado para inyectar en FreeRTOS
    static bool encolarSobre(const TxEnvelope_t& sobre);

public:
    // Inicialización (se llama en el setup)
    static void inicializar(RingbufHandle_t ringbuf);

    // Acciones públicas para enviar al GSE
    static bool enviarTelemetria(const data_all_t& datos);
    static bool enviarMensaje(const char* mensaje);
    static bool enviarError(const char* error);
    static bool enviarAcuseComando(uint32_t codigoOp, int8_t estado, float datoOpcional);
};

#endif // ENLACE_GSE_H