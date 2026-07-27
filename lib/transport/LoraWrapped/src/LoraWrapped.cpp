#include "LoraWrapped.h"

#include "data.h"


// Pasamos las variables del constructor directo a RadioLib
LoraWrapped::LoraWrapped(uint32_t nss, uint32_t rst, uint32_t pin3, uint32_t pin4, SPIClass& spi) {
    _st = CONNECTION_STATUS::DISCONNECTED;
    _pinPacketReady = pin3; 

    #if defined(MODULE_SX1278)
        _mod = new Module(nss, pin3, rst, pin4, spi);
        if(_mod){
            ESP_LOGI("CONSTRUCTOR", "MODULO INICIALIZADO");
        }
        else{
            ESP_LOGI("CONSTRUCTOR", "MODULO ERROR");
        }
        _radio = new SX1278(_mod);
        if(_radio){
            ESP_LOGI("CONSTRUCTOR", "RADIO INICIALIZADO");
        }
        else{
            ESP_LOGI("CONSTRUCTOR", "RADIO ERROR");
        }
    #elif defined(MODULE_SX1262)
        // Ahora sí, pin4 acepta RADIOLIB_NC (-1) de forma segura sin overflow
        _mod = new Module(nss, pin3, rst, pin4, spi); 
        _radio = new SX1262(_mod);
    #endif
}

LoraWrapped::~LoraWrapped()
{
    if (_mod)   delete _mod;
}

bool LoraWrapped::begin(int sw, char ew, float frec){
    this->_encryptWord = ew;
    this->_syncWord = sw;

    // Inicializar el chip físico
    int16_t state = _radio->begin(frec);
    if (state == RADIOLIB_ERR_NONE) {
        _radio->setSyncWord(sw);
        
        // CRUCIAL: Poner el radio en modo escucha asíncrona permanente
        _radio->startReceive(); 
        return true;
    }
    ESP_LOGE("LORA WRAPPED", "Radio lib no pudo inicializar, codigo de error: %d", state);
    return false;
}

bool LoraWrapped::_send_packet(pkt_t *ptrPkt) const {
    if (ptrPkt == nullptr) {
        ESP_LOGE("SENDPACKET", "PUNTERO NULO");
        return false;
    }

    // Validación estricta: el tamaño no puede superar la estructura más grande permitida
    constexpr size_t MAX_PACKET_SIZE = sizeof(data_all_t);
    if (ptrPkt->len > MAX_PACKET_SIZE) {
        ESP_LOGE("LORA", "Error: Intento de enviar paquete mayor al permitido (%d bytes)", ptrPkt->len);
        return false;
    }

    // Búfer dimensionado de forma segura para soportar tanto strings como data_all_t
    static uint8_t txBuffer[MAX_PACKET_SIZE + 2];
        memset((void*)txBuffer, 0, sizeof(txBuffer));
        
    txBuffer[0] = _encrypt_byte(ptrPkt->len);
    txBuffer[1] = _encrypt_byte(ptrPkt->protocol);

    // Encriptación y copia segura
    for (size_t i = 0; i < ptrPkt->len; i++) {
        txBuffer[2 + i] = _encrypt_byte(((uint8_t*)ptrPkt->payload)[i]);
    }

    // RadioLib transmite todo el búfer de un solo golpe
    const int state = _radio->transmit(txBuffer, ptrPkt->len + 2);
    ESP_LOGI("SENDPACKET", "---> estado: %i", state);
    // OBLIGATORIO: Volver a activar el modo escucha inmediatamente después de transmitir
    _radio->startReceive();

    return (state == RADIOLIB_ERR_NONE);
}

bool LoraWrapped::_read_packet(pkt_t *ptrPkt) {
    if (ptrPkt == nullptr) return false;

    // Consulta el pin físico de interrupción para saber si realmente hay un paquete en el aire
    #if defined(MODULE_SX1278)
        const bool packetReady = (digitalRead(_pinPacketReady) == HIGH);
    #elif defined(MODULE_SX1262)
        bool packetReady = (digitalRead(_pinPacketReady) == HIGH);
    #endif

    if (!packetReady)
        return false;

    // Lee la longitud real del paquete físico recibido
    const size_t length = _radio->getPacketLength();
    
    // El tamaño máximo permitido no puede superar la estructura más grande (data_all_t) más los 2 bytes de cabecera
    constexpr size_t max_allowed_len = sizeof(data_all_t);
    
    if (length < 2 || length > (max_allowed_len + 2)) {
        // Paquete mal formado o de un tamaño no soportado: descartamos y liberamos radio
        _radio->startReceive();
        return false;
    }

    // Búfer dimensionado de forma segura para soportar la estructura completa de telemetría
    static uint8_t rxBuffer[max_allowed_len + 2];
    memset((void*)rxBuffer, 0, sizeof(rxBuffer));
    // Lectura segura sin riesgo de desbordamiento de pila
    const int state = _radio->readData(rxBuffer, length);
    
    // OBLIGATORIO: Volver a activar la escucha de inmediato
    _radio->startReceive();

    if (state != RADIOLIB_ERR_NONE)
        return false;

    // Desencripta encabezados primarios
    ptrPkt->len = _encrypt_byte(static_cast<char>(rxBuffer[0]));
    ptrPkt->protocol = _encrypt_byte(static_cast<char>(rxBuffer[1]));

    // Validaciones estrictas de integridad
    if (ptrPkt->len > max_allowed_len || ptrPkt->len != (length - 2)) {
        return false;
    }

    // Asigna el puntero de la unión según el protocolo 
    if (ptrPkt->protocol == lora_protocol::C_PLOT) {
        memset((void*)&_internalPayload_rx.data, 0, sizeof(data_all_t));
        ptrPkt->payload = (data_all_t*)&_internalPayload_rx.data;
    } else {
        memset((void*)_internalPayload_rx.msg, 0, sizeof(_internalPayload_rx.msg));
        ptrPkt->payload = static_cast<char *>(_internalPayload_rx.msg);
    }

    // Desencriptar y rellenar el payload final de forma segura
    for (size_t i = 0; i < ptrPkt->len; i++) {
        static_cast<uint8_t *>(ptrPkt->payload)[i] = _encrypt_byte(static_cast<char>(rxBuffer[2 + i]));
    }

    return true;
}
/*
bool LoraWrapped::_send_packet(pkt_t *ptrPkt) const {
    if (ptrPkt == nullptr) return false;

    // Crea un búfer temporal para consolidar la trama completa
    uint8_t txBuffer[SIZE_BUFFER_MSG + 2];
    txBuffer[0] = _encrypt_byte(ptrPkt->len);
    txBuffer[1] = _encrypt_byte(ptrPkt->protocol);

    // Encriptación
    for (int i = 0; i < ptrPkt->len; i++) {
        txBuffer[2 + i] = _encrypt_byte(((uint8_t*)ptrPkt->payload)[i]);
    }

    // RadioLib transmite todo el búfer de un solo golpe
    const int state = _radio->transmit(txBuffer, ptrPkt->len + 2);

    // OBLIGATORIO: Volver a activar el modo escucha inmediatamente después de transmitir
    _radio->startReceive();

    return (state == RADIOLIB_ERR_NONE);
}
*/

/*
bool LoraWrapped::_read_packet(pkt_t *ptrPkt) {
    if (ptrPkt == nullptr) return false;

    // Consulta el pin físico de interrupción para saber si realmente hay un paquete en el aire
    #if defined(MODULE_SX1278)
        const bool packetReady = (digitalRead(_pinPacketReady) == HIGH);
    #elif defined(MODULE_SX1262)
        bool packetReady = (digitalRead(_pinPacketReady) == HIGH);
    #endif

    if (!packetReady)
        return false;

    // Lee la longitud del paquete recibido y vuelca los datos
    const size_t length = _radio->getPacketLength();
    uint8_t rxBuffer[SIZE_BUFFER_MSG + 2];

    const int state = _radio->readData(rxBuffer, length);
    
    // Vuelve a activar la escucha de inmediato para no perder paquetes futuros
    _radio->startReceive();

    if (state != RADIOLIB_ERR_NONE || length < 2)
        return false;

    // Desencripta encabezados primarios
    ptrPkt->len = _encrypt_byte(static_cast<char>(rxBuffer[0]));
    ptrPkt->protocol = _encrypt_byte(static_cast<char>(rxBuffer[1]));

    if (ptrPkt->len > SIZE_BUFFER_MSG) return false;

    // Asigna el puntero de la unión según el protocolo 
    if (ptrPkt->protocol == lora_protocol::C_PLOT) {
        memset((void*)&_internalPayload_rx.data, 0, sizeof(data_all_t));
        ptrPkt->payload = (data_all_t*)&_internalPayload_rx.data;
    } else {
        memset((void*)_internalPayload_rx.msg, 0, sizeof(_internalPayload_rx.msg));
        ptrPkt->payload = static_cast<char *>(_internalPayload_rx.msg);
    }

    // Desencriptar y rellenar el payload final
    for (int i = 0; i < ptrPkt->len; i++) {
        static_cast<uint8_t *>(ptrPkt->payload)[i] = _encrypt_byte(static_cast<char>(rxBuffer[2 + i]));
    }

    return true;
}
*/
bool LoraWrapped::c_connect_to_GSE() const {
    pkt_t packet;
    const char *msg = "PING_COHETE"; // TODO: redundante
    
    // Prepara el paquete
    packet.protocol = lora_protocol::PING;
    packet.payload = (void*)msg;
    packet.len = strlen(msg) + 1; // envía solo los bytes necesarios

    return _send_packet(&packet);
}


bool LoraWrapped::g_accept_connection(){
    pkt_t rx_packet;
    pkt_t tx_packet;
    // mensaje de respuesta(PONG)
    const char* respuesta = "CONEXION_ACEPTADA";

    // Intenta leer un paquete siguiendo la lógica de la fachada
    if (!read_packet(&rx_packet)) {
        return false;
    }

    if( rx_packet.protocol == lora_protocol::PING){
        // Verifica la integridad del mensaje // TODO: redundante. se puede simplificar el ping pong usando solamente C_PING y C_PONG.
        if(strcmp((char*)rx_packet.payload, "PING_COHETE") != 0){
            return false;
        } 

        // Prepara la respuesta
        tx_packet.protocol = lora_protocol::PONG;
        tx_packet.payload = (void*)respuesta;
        tx_packet.len = strlen(respuesta) + 1;

        // Envia la confirmación
        _st = CONNECTION_STATUS::CONNECTED;
        return _send_packet(&tx_packet);
    }
    return false;
}


bool LoraWrapped::c_connection_accepted() {
    pkt_t packet;
    
    // Verificamos si llegó algo
    if (_read_packet(&packet)) {
        // El cohete espera un PONG para confirmar la conexión
        if (packet.protocol == lora_protocol::PONG) {
            // un flag de "ENLACE COMPLETADO"
            _st = CONNECTION_STATUS::CONNECTED;
            return true;
        }
    }
    return false;
}


// TODO: CAMBIAR data_all_t por una struct propia de la GSE.
// TODO: datos debería ser data_all_t o data_all_t* ???
bool LoraWrapped::send_data(data_all_t datos) const {
    pkt_t packet;

    // Verifica conexion
    if(_st !=CONNECTION_STATUS::CONNECTED)
        return false;
    
    // Prepara el paquete
    packet.protocol = lora_protocol::C_PLOT;
    packet.payload = &datos;
    packet.len = sizeof(datos);

    // Envia el paquete
    return _send_packet(&packet);
}


bool LoraWrapped::send_msg(const char* texto) const {
    pkt_t packet;

    // verifica la conexion o si existe el mensaje
    if (texto == nullptr || _st !=CONNECTION_STATUS::CONNECTED)
        return false;


    // Configura el paquete de mensaje
    packet.protocol = lora_protocol::C_MGS;
    packet.payload = (void*)texto;
    packet.len = strlen(texto) + 1; // +1 para incluir el '\0'

    return _send_packet(&packet);
}


bool LoraWrapped::send_error(const char* error) const {
    pkt_t packet;

    if (error == nullptr || _st !=CONNECTION_STATUS::CONNECTED)
        return false;

    // Configuramos el paquete como error
    packet.protocol = lora_protocol::C_ERR;
    packet.payload = (void*)error;
    packet.len = strlen(error) + 1;

    return _send_packet(&packet);
}

bool LoraWrapped::send_pong() {
    pkt_t rx_packet;
    // mensaje de respuesta(PONG)
    const char* respuesta = "CONEXION_ACEPTADA";
    // Prepara la respuesta
    rx_packet.protocol = lora_protocol::PONG;
    rx_packet.payload = (void*)respuesta;
    rx_packet.len = strlen(respuesta) + 1;

    // Envia la confirmación
    _st = CONNECTION_STATUS::CONNECTED;
    return _send_packet(&rx_packet);
}

bool LoraWrapped::g_send_command(CommandPayload comando)
{
    pkt_t packet;
    // Verifica conexion
    // if(_st !=CONNECTION_STATUS::CONNECTED)
    //     return false;
    
    // Prepara el paquete
    packet.protocol = lora_protocol::G_CMD;
    packet.payload = &comando;
    packet.len = sizeof(comando);

    // Envia el paquete
    return _send_packet(&packet);

}

bool LoraWrapped::c_send_ACK(CmdAck_t ACK)
{
    static CmdAck_t ack = ACK;
    pkt_t packet = {};
    
    // Prepara el paquete
    packet.protocol = lora_protocol::C_ACK;
    packet.payload = (void*)&ack;
    packet.len = sizeof(ack); // envía solo los bytes necesarios

    return _send_packet(&packet);
}

bool LoraWrapped::read_packet(pkt_t *pPkt)
{
    if(pPkt == nullptr /*||_st != CONNECTION_STATUS::CONNECTED*/)
        return false;


    return _read_packet(pPkt);
}
