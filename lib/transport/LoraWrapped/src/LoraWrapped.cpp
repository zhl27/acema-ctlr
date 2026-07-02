#include "LoraWrapped.h"

#include "data.h"


// Pasamos las variables del constructor directo a RadioLib
LoraWrapped::LoraWrapped(uint32_t nss, uint32_t rst, uint32_t pin3, uint32_t pin4, SPIClass& spi) {
    _st = CONNECTION_STATUS::DISCONNECTED;
    _pinPacketReady = pin3; 

    #if defined(MODULE_SX1278)
        _mod = new Module(nss, pin3, rst, pin4, spi);
        _radio = new SX1278(_mod);
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

int LoraWrapped::begin(int sw, char ew, float frec){
    this->_encryptWord = ew;
    this->_syncWord = sw;

    // Inicializar el chip físico
    int state = _radio->begin(frec);
    if (state == RADIOLIB_ERR_NONE) {
        _radio->setSyncWord(sw);
        
        // CRUCIAL: Poner el radio en modo escucha asíncrona permanente
        _radio->startReceive(); 
        return state;
    }
    return state;
}


int LoraWrapped::_send_packet(pkt_t *ptrPkt) const {
    if (ptrPkt == nullptr) return RADIOLIB_ERR_UNKNOWN;

    // Crea un búfer temporal para consolidar la trama completa
    uint8_t txBuffer[SIZE_BUFFER_MSG + 2];
    txBuffer[0] = _encrypt_byte(ptrPkt->len);
    txBuffer[1] = _encrypt_byte(ptrPkt->protocole);

    // Encriptación
    for (int i = 0; i < ptrPkt->len; i++) {
        txBuffer[2 + i] = _encrypt_byte(((uint8_t*)ptrPkt->payload)[i]);
    }

    // RadioLib transmite todo el búfer de un solo golpe
    const int state = _radio->transmit(txBuffer, ptrPkt->len + 2);

    // OBLIGATORIO: Volver a activar el modo escucha inmediatamente después de transmitir
    _radio->startReceive();

    return state;
}


int LoraWrapped::_read_packet(pkt_t *ptrPkt) {
    if (ptrPkt == nullptr) return RADIOLIB_ERR_UNKNOWN; //TODO: PENSAR UN RETORNO DE ERROR MÁS REPRESENTATIVO PARA NUESTRO LORAWRAPPED

    // Consulta el pin físico de interrupción para saber si realmente hay un paquete en el aire
    #if defined(MODULE_SX1278)
        const bool packetReady = (digitalRead(_pinPacketReady) == HIGH);
    #elif defined(MODULE_SX1262)
        bool packetReady = (digitalRead(_pinPacketReady) == HIGH);
    #endif

    if (!packetReady) return RADIOLIB_ERR_UNKNOWN;

    // Lee la longitud del paquete recibido y vuelca los datos
    const size_t length = _radio->getPacketLength();
    uint8_t rxBuffer[SIZE_BUFFER_MSG + 2];

    const int state = _radio->readData(rxBuffer, length);
    
    // Vuelve a activar la escucha de inmediato para no perder paquetes futuros
    _radio->startReceive();

    if (state != RADIOLIB_ERR_NONE || length < 2)
        return state;

    // Desencripta encabezados primarios
    ptrPkt->len = _encrypt_byte(static_cast<char>(rxBuffer[0]));
    ptrPkt->protocole = _encrypt_byte(static_cast<char>(rxBuffer[1]));

    if (ptrPkt->len > SIZE_BUFFER_MSG) return false;

    // Asigna el puntero de la unión según el protocolo 
    if (ptrPkt->protocole == lora_protocol::C_PLOT) {
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

    return state; //TODO: revisar si este "state" es representativo del resultado de esta funcion ("_read_packet")
}

int LoraWrapped::c_connect_to_GSE() const {
    pkt_t paquete;
    const char *msg = "PING_COHETE"; // TODO: redundante
    
    // Prepara el paquete
    paquete.protocole = lora_protocol::PING;
    paquete.payload = (void*)msg;
    paquete.len = strlen(msg) + 1; // envía solo los bytes necesarios

    return _send_packet(&paquete);
}


int LoraWrapped::g_accept_connection(){
    pkt_t paqueteRecibido;
    pkt_t paqueteRespuesta;
    // mensaje de respuesta(PONG)

    // Intenta leer un paquete siguiendo la lógica de la fachada
    int16_t res = _read_packet(&paqueteRecibido);
    if (!res) { // si hay error, retornar código de error
        return res;
    }
    
    if( paqueteRecibido.protocole == lora_protocol::PING){
        const char* respuesta = "CONEXION_ACEPTADA";
        // Verifica la integridad del mensaje // TODO: redundante. se puede simplificar el ping pong usando solamente C_PING y C_PONG.
        if(strcmp((char*)paqueteRecibido.payload, "PING_COHETE") != 0){
            return RADIOLIB_ERR_UNKNOWN;
        } 

        // Prepara la respuesta
        paqueteRespuesta.protocole = lora_protocol::PONG;
        paqueteRespuesta.payload = (void*)respuesta;
        paqueteRespuesta.len = strlen(respuesta) + 1;

        // Envia la confirmación
        _st = CONNECTION_STATUS::CONNECTED;
        return _send_packet(&paqueteRespuesta);
    }
    return RADIOLIB_ERR_UNKNOWN;
}


int LoraWrapped::c_connection_accepted() {
    pkt_t paquete;
    
    // Verificamos si llegó algo
    if (_read_packet(&paquete)) {
        // El cohete espera un PONG para confirmar la conexión
        if (paquete.protocole == lora_protocol::PONG) {
            // un flag de "ENLACE COMPLETADO"
            _st = CONNECTION_STATUS::CONNECTED;
            return _st; //TODO: REVISAR QUE NO SE SOLAPE CON ENUMS DE RADIOLIB
        }
    }
    return RADIOLIB_ERR_UNKNOWN;
}


// TODO: CAMBIAR data_all_t por una struct propia de la GSE.
// TODO: datos debería ser data_all_t o data_all_t*
int LoraWrapped::send_data(data_all_t datos) const {
    pkt_t paquete;

    // Verifica conexion
    if(_st != CONNECTION_STATUS::CONNECTED)
        return _st;
    
    // Prepara el paquete
    paquete.protocole = lora_protocol::C_PLOT;
    paquete.payload = &datos;
    paquete.len = sizeof(datos);

    // Envia el paquete
    return _send_packet(&paquete);
}


int LoraWrapped::send_msg(const char* texto) const {
    pkt_t paquete;

    // verifica la conexion o si existe el mensaje
    if (texto == nullptr || _st !=CONNECTION_STATUS::CONNECTED)
        return _st;



    // Configura el paquete de mensaje
    paquete.protocole = lora_protocol::C_MGS;
    paquete.payload = (void*)texto;
    paquete.len = strlen(texto) + 1; // +1 para incluir el '\0'

    return _send_packet(&paquete);
}


int LoraWrapped::send_error(const char* error) const {
    pkt_t paquete;

    if (error == nullptr || _st !=CONNECTION_STATUS::CONNECTED)
        return _st;

    // Configuramos el paquete como error
    paquete.protocole = lora_protocol::C_ERR;
    paquete.payload = (void*)error;
    paquete.len = strlen(error) + 1;

    return _send_packet(&paquete);
}

int LoraWrapped::send_pong() {
    pkt_t paqueteRespuesta;
    // mensaje de respuesta(PONG)
    const char* respuesta = "CONEXION_ACEPTADA";
    // Prepara la respuesta
    paqueteRespuesta.protocole = lora_protocol::PONG;
    paqueteRespuesta.payload = (void*)respuesta;
    paqueteRespuesta.len = strlen(respuesta) + 1;

    // Envia la confirmación
    _st = CONNECTION_STATUS::CONNECTED;
    return _send_packet(&paqueteRespuesta);
}

int LoraWrapped::read_packet(pkt_t *pPkt) {
    if(pPkt == nullptr /*||_st != CONNECTION_STATUS::CONNECTED*/){
        return false;
    }

    return _read_packet(pPkt);
}
