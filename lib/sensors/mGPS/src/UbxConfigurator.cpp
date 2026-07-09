#include <UbxConfigurator.h>
#include <cstring> // Necesario para memset y memcpy


// ==========================================
// Constructor
// ==========================================
UbxConfigurator::UbxConfigurator(const TxCallback txFunc, const WaitAckCallback waitFunc) :
    _txFunc(txFunc), 
    _waitFunc(waitFunc) 
{
    // Las funciones de callback se inyectan en el momento de crear la instancia
    assert (_txFunc && _waitFunc); // ¡SÍ O SÍ!
}

// ==========================================
// Configuraciones Específicas
// ==========================================

bool UbxConfigurator::setPortUart(uint32_t baudrate) {
    cfg_prt_uart_t payload;
    memset(&payload, 0, sizeof(cfg_prt_uart_t)); // Limpieza total [cite: 622]

    payload.portID = 1; // 1 = UART 1 [cite: 623]
    
    // Configuración UART: 8 Bits de datos, 1 Stop Bit, Sin Paridad (8N1)
    // En hexadecimal según el datasheet de u-blox: 0x000008D0 [cite: 623]
    payload.mode.mode = 0x000008D0; 
    payload.baudRate = baudrate;
    
    // Máscaras de entrada/salida. Cada bit habilita un protocolo[cite: 624, 625].
    // Bit 0 = UBX, Bit 1 = NMEA, Bit 5 = RTCM.
    // Nosotros queremos UBX exclusivo, por lo tanto seteamos el bit 0 en '1' (0x0001).
    payload.inProtoMask.inProtoMask = 0x0001; 
    payload.outProtoMask.outProtoMask = 0x0001; 

    // Clase 0x06 (CFG), ID 0x00 (PRT) [cite: 617, 622]
    return _buildAndSend(UBX_CLASS::CFG, static_cast<uint8_t>(UBX_ID_CFG::PRT), (uint8_t*)&payload, sizeof(cfg_prt_uart_t));
}

bool UbxConfigurator::setNavigationRate(uint8_t rateHz) {
    cfg_rate_t payload;
    // Evitamos división por cero o Frecuencia max del módulo
    if (rateHz == 0 || rateHz > 10) return false; 
    
    memset(&payload, 0, sizeof(cfg_rate_t));

    // El GPS espera el periodo en milisegundos [cite: 631]
    // Ejemplo: 5 Hz -> 1000 / 5 = 200 ms
    payload.measRate = 1000 / rateHz; 
    payload.navRate = 1; // 1 medición por ciclo de navegación [cite: 631]
    payload.timeRef = 1; // 1 = Alineado a tiempo GPS (0 = UTC) [cite: 631]

    // Clase 0x06 (CFG), ID 0x08 (RATE) [cite: 631]
    return _buildAndSend(UBX_CLASS::CFG, static_cast<uint8_t>(UBX_ID_CFG::RATE), (uint8_t*)&payload, sizeof(cfg_rate_t));
}

bool UbxConfigurator::setDynamicModel(nav_dyn_model_e model) {
    cfg_nav5_t payload;
    memset(&payload, 0, sizeof(cfg_nav5_t));

    // Solo queremos modificar el modelo dinámico, le pasamos la máscara específica (Bit 0) [cite: 612]
    payload.mask = 0x0001; 
    payload.dynModel = static_cast<uint8_t>(model);

    // Clase 0x06 (CFG), ID 0x24 (NAV5) [cite: 611]
    return _buildAndSend(UBX_CLASS::CFG, static_cast<uint8_t>(UBX_ID_CFG::NAV5), (uint8_t*)&payload, sizeof(cfg_nav5_t));
}


bool UbxConfigurator::enableRegisteredMessages(const UbxRegMsg_t **ptrTablaMsg, const size_t tamanioTabla) {
    bool allSuccess = true;

    for (size_t i = 0; i < tamanioTabla; i++) {
        uint8_t cls = ptrTablaMsg[i]->msgClass;
        uint8_t id = ptrTablaMsg[i]->msgID;

        // No tiene sentido pedirle al GPS que nos envíe periódicamente un ACK
        if (cls == static_cast<uint8_t>(UBX_CLASS::ACK)) {
            continue; 
        }

        cfg_msg_t msgPayload;
        msgPayload.msgClass = cls;
        msgPayload.msgID = id;
        msgPayload.rate = 1; // Enviar en cada ciclo

        // Enviamos el comando UBX-CFG-MSG (0x06 0x01)
        bool success = _buildAndSend(UBX_CLASS::CFG, static_cast<uint8_t>(UBX_ID_CFG::MSG), (uint8_t*)&msgPayload, sizeof(cfg_msg_t));
        
        if (!success) {
            allSuccess = false; // Registramos si falló alguno
        }
    }

    return allSuccess;
}

// ==========================================
// Ensamblador y Motor de Sincronización
// ==========================================

bool UbxConfigurator::_buildAndSend(ubx_class_e msgClass, uint8_t msgID, const uint8_t* payload, size_t payloadSize) const {
    const size_t MAX_PACKET_SIZE = 128;
    size_t packetSize;
    uint8_t buffer[MAX_PACKET_SIZE];
    uint8_t ckA = 0, ckB = 0;
    
    // Validamos que el usuario haya inyectado los callbacks RTOS
    if (!_txFunc || !_waitFunc) return false;

    // 128 bytes es suficiente. CFG-NAV5 pesa 36 bytes [cite: 611], el más pesado es CFG-USB de 108 bytes 

    packetSize = 2 + 4 + payloadSize + 2; // Sync(2) + Header(4) + Payload(N) + CRC(2)

    if (packetSize > MAX_PACKET_SIZE) return false; // Protección de memoria

    // 1. Caracteres de Sincronización
    buffer[0] = static_cast<uint8_t>(UBX_SYNC::SYNC_1);
    buffer[1] = static_cast<uint8_t>(UBX_SYNC::SYNC_2);

    // 2. Encabezado (Little Endian)
    buffer[2] = static_cast<uint8_t>(msgClass);
    buffer[3] = msgID;
    buffer[4] = payloadSize & 0xFF; // Solo asigna el primer byte
    buffer[5] = (payloadSize >> 8) & 0xFF;

    // 3. Copiado de Payload
    if (payloadSize > 0 && payload != nullptr) {
        memcpy(&buffer[6], payload, payloadSize);
    }

    // 4. Cálculo del Checksum (Fletcher-8) sobre Encabezado + Payload

    for (size_t i = 2; i < 6 + payloadSize; i++) {
        ckA += buffer[i];
        ckB += ckA;
    }

    buffer[6 + payloadSize] = ckA;
    buffer[7 + payloadSize] = ckB;

    // 5. Inyectar en el hardware
    // Llamamos a la función inyectada (Ej: uart_write_bytes en el ESP32)
    _txFunc(buffer, packetSize);

    // 6. Bloqueo RTOS esperando respuesta
    // Le decimos a la capa superior: "Dormime hasta recibir un ACK para esta Clase e ID. Time-out en 1500ms"
    return _waitFunc(static_cast<uint8_t>(msgClass), msgID, 1500);
}
