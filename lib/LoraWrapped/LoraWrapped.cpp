#include "LoraWrapped.h"



LoraWrapped::LoraWrapped(int ss, int reset, int dio0, SPIClass& spi)
{
    _st = CONNECTION_STATUS::DISCONNECTED;
    
    // Inicialización condicional del objeto de RadioLib según el chip elegido
    #if defined(MODULE_SX1278)
        _mod = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1);
        _radio = new SX1278(_mod);
    #elif defined(MODULE_SX1262)
        _mod = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
        _radio = new SX1262(_mod);
    #endif
}

LoraWrapped::~LoraWrapped()
{
    delete _radio;
    delete _mod;
}

bool LoraWrapped::begin(int sw, char ew, float frec){
    this->_encryptWord = ew;
    this->_syncWord = sw;

    // Inicializar el chip físico
    int state = _radio->begin(frec);
    if (state == RADIOLIB_ERR_NONE) {
        _radio->setSyncWord(sw);
        
        // CRUCIAL: Poner el radio en modo escucha asíncrona permanente
        _radio->startReceive(); 
        return true;
    }
    return false;
}

// PARALELISMO: TPC
/*
* Para mandar un paquete se debe enviar de manera ordenada la siguiente trama:
* len (1 byte) -> protocol (1 byte) -> payload (len bytes)
* protocolo entre Lora's 
* paquete = (len, proto, payload)
*/


// HTTP
/*
	protocolo entre ESP_gse <-> compu (gse)
	esp del gse recibe la struct (bytes), y lo manda por serial con este formato:
	
	 '$'<Bytes(paquete 100)>'#'     '$'<Bytes(paquete 30)>'#'      '$'<Bytes(paquete 30)>'#'       '$'<Bytes(pa quete 80)>'#'  
	
	// IMPLEMENTACION SEGURA CON MAQUINA DE ESTADOS
	cohete -> gse: '$''<long N>'
	esperando_pesos -- si Byte[0] == '$' && byte[1] > 0  / envio 'ok'  -> recibiendo N (leer N bytes)
	                                             <---
	gse(serial) -> 'ok'
	
	enviar_pesos --> esperar 'ok' (leyendo sensor, mde, control vuelo) --> enviar N bytes (envio siguiente dato)
					<---(vuelvo a mandarel mismo)
	cohete -> bytes
	
	trama= seial.leerHasta('#')
	SI trama[0] == '$'
	long
	[1], proto, = pkt
	48 -> datos plot,
*/


bool LoraWrapped::send_package(pkt_t * ptrPkt){
    if (ptrPkt == nullptr) return false;

    // Crea un búfer temporal para consolidar la trama completa
    uint8_t txBuffer[SIZE_BUFFER_MSG + 2];
    txBuffer[0] = encrypt_byte(ptrPkt->len);
    txBuffer[1] = encrypt_byte(ptrPkt->protocole);

    // Encriptación
    for (int i = 0; i < ptrPkt->len; i++) {
        txBuffer[2 + i] = encrypt_byte(((uint8_t*)ptrPkt->payload)[i]);
    }

    // RadioLib transmite todo el búfer de un solo golpe
    int state = _radio->transmit(txBuffer, ptrPkt->len + 2);

    // OBLIGATORIO: Volver a activar el modo escucha inmediatamente después de transmitir
    _radio->startReceive();

    return (state == RADIOLIB_ERR_NONE);
}


bool LoraWrapped::read_package(pkt_t *ptrPkt) {
    if (ptrPkt == nullptr) return false;

    // Consulta el pin físico de interrupción para saber si realmente hay un paquete en el aire
    #if defined(MODULE_SX1278)
        bool packetReady = (digitalRead(LORA_DIO0) == HIGH);
    #elif defined(MODULE_SX1262)
        bool packetReady = (digitalRead(LORA_DIO1) == HIGH);
    #endif

    if (!packetReady) return false;

    // Lee la longitud del paquete recibido y vuelca los datos
    size_t length = _radio->getPacketLength();
    uint8_t rxBuffer[SIZE_BUFFER_MSG + 2];
    
    int state = _radio->readData(rxBuffer, length);
    
    // Vuelve a activar la escucha de inmediato para no perder paquetes futuros
    _radio->startReceive();

    if (state != RADIOLIB_ERR_NONE || length < 2) return false;

    // Desencripta encabezados primarios
    ptrPkt->len = encrypt_byte((char)rxBuffer[0]);
    ptrPkt->protocole = encrypt_byte((char)rxBuffer[1]);

    if (ptrPkt->len > SIZE_BUFFER_MSG) return false;

    // Asigna el puntero de la unión según el protocolo 
    if (ptrPkt->protocole == Protocolo::C_PLOT) {
        memset((void*)&_internalPayload_rx.data, 0, sizeof(dataPlot_t));
        ptrPkt->payload = (dataPlot_t*)&_internalPayload_rx.data;
    } else {
        memset((void*)_internalPayload_rx.msg, 0, sizeof(_internalPayload_rx.msg));
        ptrPkt->payload = (char*)_internalPayload_rx.msg;
    }

    // Desencriptar y rellenar el payload final
    for (int i = 0; i < ptrPkt->len; i++) {
        ((uint8_t*)ptrPkt->payload)[i] = encrypt_byte((char)rxBuffer[2 + i]);
    }

    return true;
}

bool LoraWrapped::c_connect_to_GSE(){
    pkt_t paquete;
    const char *msg = "PING_COHETE";
    
    // Prepara el paquete
    paquete.protocole = Protocolo::PING;
    paquete.payload = (void*)msg;
    paquete.len = strlen(msg) + 1; // envía solo los bytes necesarios

    return send_package (&paquete);
}


bool LoraWrapped::g_accept_connection(){
    pkt_t paqueteRecibido;
    pkt_t paqueteRespuesta;
    // mensaje de respuesta(PONG)
    const char* respuesta = "CONEXION_ACEPTADA";
    
    // Intenta leer un paquete siguiendo la lógica de la fachada
    if (!read_package(&paqueteRecibido)) {
        return false;
    }
    
    if( paqueteRecibido.protocole == Protocolo::PING){
        // Verifica la integridad del mensaje
        if(strcmp((char*)paqueteRecibido.payload, "PING_COHETE") != 0){
            return false;
        } 

        // Prepara la respuesta
        paqueteRespuesta.protocole = Protocolo::PONG;
        paqueteRespuesta.payload = (void*)respuesta;
        paqueteRespuesta.len = strlen(respuesta) + 1;

        // Envia la confirmación
        _st = CONNECTION_STATUS::CONNECTED;
        return send_package(&paqueteRespuesta);
    }
    return false;
}


bool LoraWrapped::c_connection_accepted() {
    pkt_t paquete;
    
    // Verificamos si llegó algo
    if (read_package(&paquete)) {
        // El cohete espera un PONG para confirmar la conexión
        if (paquete.protocole == Protocolo::PONG) {
            // un flag de "ENLACE COMPLETADO"
            _st = CONNECTION_STATUS::CONNECTED;
            return true;
        }
    }
    return false;
}



bool LoraWrapped::send_datos(dataPlot_t datos) {
    pkt_t paquete;

    // Verifica conexion
    if(_st !=CONNECTION_STATUS::CONNECTED) return false;
    
    // Prepara el paquete
    paquete.protocole = Protocolo::C_PLOT;
    paquete.payload = &datos;
    paquete.len = sizeof(dataPlot_t);

    // Envia el paquete
    return send_package(&paquete);
}


bool LoraWrapped::send_mensaje(const char* texto) {
    pkt_t paquete;

    // verifica la conexion o si existe el mensaje
    if (texto == nullptr || _st !=CONNECTION_STATUS::CONNECTED){
        return false;
    }


    // Configura el paquete de mensaje
    paquete.protocole = Protocolo::C_MGS; 
    paquete.payload = (void*)texto;
    paquete.len = strlen(texto) + 1; // +1 para incluir el '\0'

    return send_package(&paquete);
}


bool LoraWrapped::send_mensaje_error(const char* error) {
    pkt_t paquete;

    if (error == nullptr || _st !=CONNECTION_STATUS::CONNECTED) return false;

    // Configuramos el paquete como error
    paquete.protocole = Protocolo::C_ERR;
    paquete.payload = (void*)error;
    paquete.len = strlen(error) + 1;

    return send_package(&paquete);
}

bool LoraWrapped::read_paquete(pkt_t *pPkt)
{
    if(pPkt == nullptr ||_st != CONNECTION_STATUS::CONNECTED){
        return false;
    }

    return read_package(pPkt);
}
