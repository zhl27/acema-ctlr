#include "EnlaceGSE.h"
#include "esp_log.h"

// Inicializar el miembro estático
RingbufHandle_t EnlaceGSE::_ringbuf = nullptr;

void EnlaceGSE::inicializar(RingbufHandle_t ringbuf) {
    _ringbuf = ringbuf;
}

bool EnlaceGSE::encolarSobre(const TxEnvelope_t& sobre) {
    if (_ringbuf == nullptr) return false;
    
    // pdMS_TO_TICKS(0) hace que el envío sea NO bloqueante. 
    // Si la cola de LoRa se llena, simplemente retorna false y no frena la tarea actual.
    return xRingbufferSend(_ringbuf, (void*)&sobre, sizeof(TxEnvelope_t), 0) == pdTRUE;
}

bool EnlaceGSE::enviarTelemetria(const data_all_t& datos) {
    TxEnvelope_t sobre = {}; // Llena todo con ceros inicialmente
    sobre.tipo = lora_protocol::C_PLOT;
    sobre.payload.telemetria = datos;
    
    return encolarSobre(sobre);
}

bool EnlaceGSE::enviarMensaje(const char* mensaje) {
    if (!mensaje) return false;

    TxEnvelope_t sobre = {};
    sobre.tipo = lora_protocol::C_MGS;
    
    // strncpy protege contra desbordamientos si el string es mayor a 128 caracteres.
    strncpy(sobre.payload.texto, mensaje, sizeof(sobre.payload.texto) - 1);
    // Garantizamos que siempre termine en caracter nulo
    sobre.payload.texto[sizeof(sobre.payload.texto) - 1] = '\0';

    ESP_LOGI("EnlaceGSE", "%s", mensaje);

    return encolarSobre(sobre);
}

bool EnlaceGSE::enviarError(const char* error) {
    if (!error) return false;

    TxEnvelope_t sobre = {};
    sobre.tipo = lora_protocol::C_ERR;
    
    strncpy(sobre.payload.texto, error, sizeof(sobre.payload.texto) - 1);
    sobre.payload.texto[sizeof(sobre.payload.texto) - 1] = '\0';
    
    return encolarSobre(sobre);
}

bool EnlaceGSE::enviarAcuseComando(uint32_t codigoOp, int8_t estado, float datoOpcional) {
    TxEnvelope_t sobre = {};
    sobre.tipo = lora_protocol::C_ACK;
    sobre.payload.ack.opCode = codigoOp;
    sobre.payload.ack.status = estado;
    sobre.payload.ack.datoOpcional = datoOpcional;
    
    return encolarSobre(sobre);
}