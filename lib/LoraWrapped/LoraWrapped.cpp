#include <LoraWrapped.h>



LoraWrapped::LoraWrapped(int ss, int reset, int dio0, SPIClass& spi)
{
    LoRa.setPins(ss, reset, dio0);
    LoRa.setSPI(spi);
    _st = CONNECTION_STATUS::DISCONNECTED;
}

LoraWrapped::~LoraWrapped()
{
}

bool LoraWrapped::begin(int sw, char ew, long frec){
    this->_encryptWord = ew;
    if(LoRa.begin(frec)) {
        LoRa.setSyncWord(sw);
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
    char encryptedByte = 0x00;
    
    if (ptrPkt == nullptr || !LoRa.beginPacket()){
        return false; // Lora ocupado o puntero nulo
    }

    // primer byte: longitud
    LoRa.write(encrypt_byte(ptrPkt->len));
    // segundo byte: protocol
    LoRa.write(encrypt_byte(ptrPkt->protocole));

    // len's bytes, correspondientes a la longitud del playload
    for (int i = 0; i < ptrPkt->len; i++) {
        encryptedByte = encrypt_byte((char) (((uint8_t*)ptrPkt->payload)[i]));
        LoRa.write(encryptedByte);
    }

    return (LoRa.endPacket(false)); // Modo síncrono
}


bool LoraWrapped::read_package(pkt_t *ptrPkt){
    char decryptedByte = 0x00;
    int packetSize = LoRa.parsePacket();
    
    // minimo dos bytes
    if (packetSize < 2 || ptrPkt == nullptr) {
        return false; // No hay un puntero válido
    }

    // lee encabezados
    ptrPkt->len = encrypt_byte( (char)LoRa.read() );
    ptrPkt->protocole = encrypt_byte( (char)LoRa.read() );

    // verificación de seguridad para evitar desbordamientos
    if (ptrPkt->len > sizeof(pay_u) ){
        return false;
    }

    // Asigna el puntero de la union según protocolo
    if(ptrPkt->protocole  == C_PLOT ){
        memset( (void*)&_internalPayload.data, 0, sizeof(dataPlot_t));
        ptrPkt->payload = (dataPlot_t*) &_internalPayload.data; 
    }
    /* MSG, ERR : Mensajes de error de longitud 128 bytes, incluido el '\0' */
    else{
        memset( (void*)_internalPayload.msg, 0, SIZE_BUFFER_MSG); // 128 
        ptrPkt->payload = (char*)_internalPayload.msg;
    }
    // ptrPkt->payload = PLOT? (dataPlot_t*) &payload.data: (char*)payload.msg;
    // lectura y desencriptación directa
    for(int i = 0; i< ptrPkt->len; i++){
        if(LoRa.available()){
            ((uint8_t*)ptrPkt->payload)[i] = encrypt_byte( (char)LoRa.read() );
        }
    }

    return true;
}

bool LoraWrapped::c_connect_to_GSE(){
    pkt_t paquete;
    char *msg = "PING_COHETE";
    
    // copia el mensaje en el paquete
    strncpy(_internalPayload.msg, msg, SIZE_BUFFER_MSG);
    _internalPayload.msg[SIZE_BUFFER_MSG - 1] = '\0';
    // prepara el paquete con el protocolo ping
    paquete.protocole = Protocolo::PING;
    paquete.payload = _internalPayload.msg;
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
        strncpy(_internalPayload.msg, respuesta, SIZE_BUFFER_MSG);
        _internalPayload.msg[SIZE_BUFFER_MSG - 1] = '\0';

        paqueteRespuesta.protocole = Protocolo::PONG;
        paqueteRespuesta.payload = _internalPayload.msg;
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
    _internalPayload.data = datos;
    
    // Prepara el paquete
    paquete.protocole = Protocolo::C_PLOT;
    paquete.payload = &_internalPayload.data;
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
    // Copia el texto al buffer de la unión
    strncpy(_internalPayload.msg, texto, SIZE_BUFFER_MSG);
    _internalPayload.msg[SIZE_BUFFER_MSG - 1] = '\0'; // Asegurar cierre de cadena

    // Configura el paquete de mensaje
    paquete.protocole = Protocolo::C_MGS; 
    paquete.payload = _internalPayload.msg;
    paquete.len = strlen(_internalPayload.msg) + 1; // +1 para incluir el '\0'

    return send_package(&paquete);
}


bool LoraWrapped::send_mensaje_error(const char* error) {
    pkt_t paquete;

    if (error == nullptr || _st !=CONNECTION_STATUS::CONNECTED) return false;
    
    // Copia el error al buffer
    strncpy(_internalPayload.msg, error, SIZE_BUFFER_MSG);
    _internalPayload.msg[SIZE_BUFFER_MSG - 1] = '\0';

    // Configuramos el paquete como error
    paquete.protocole = Protocolo::C_ERR;
    paquete.payload = _internalPayload.msg;
    paquete.len = strlen(_internalPayload.msg) + 1;

    return send_package(&paquete);
}
