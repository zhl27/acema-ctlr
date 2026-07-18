//
// Created by zhl on 6/3/26.
//

#include "mGPS.h"

// Inicialización de la variable estática
mGPS* mGPS::_instance = nullptr;

mGPS::mGPS(int uartNum, int rx, int tx, uint32_t baud)
    : _uartNum(uartNum), _rxPin(rx), _txPin(tx), _baud(baud),
      _serial(new HardwareSerial(uartNum)),
      _gpsQueue(nullptr),
      // Inicializamos campos personalizados para Fix Type (campo 2) y PDOP (campo 15)
      // Soportamos tanto $GPGSA (solo GPS) como $GNGSA (GNSS multi-constelación u-blox)
      _pdop_gpgsa(_gps, "GPGSA", 15),
      _pdop_gngsa(_gps, "GNGSA", 15),
      _fix_gpgsa(_gps, "GPGSA", 2),
      _fix_gngsa(_gps, "GNGSA", 2)
{
    _instance = this;
}

mGPS::~mGPS() {
    if (_serial) {
        _serial->end();
        delete _serial;
        _serial = nullptr;
    }
    if (_gpsQueue) {
        vQueueDelete(_gpsQueue);
        _gpsQueue = nullptr;
    }
    if (_instance == this) {
        _instance = nullptr;
    }
}

bool mGPS::init() {
    // 1. Inicializar el puerto serie de hardware
    _serial->begin(_baud, SERIAL_8N1, _rxPin, _txPin);

    // 2. Crear la cola de FreeRTOS tipo "Mailbox" (longitud 1)
    _gpsQueue = xQueueCreate(1, sizeof(data_gps_t));
    if (_gpsQueue == nullptr) {
        return false;
    }

    // Inicializar el buzón con una estructura vacía y segura
    data_gps_t initial_pvt = {0};
    initial_pvt.fix_type = 1; // 1 = Sin Fix
    initial_pvt.gnss_fix_ok = false;
    initial_pvt.hdop = 99.9f;
    initial_pvt.pdop = 99.9f;
    xQueueOverwrite(_gpsQueue, &initial_pvt);

    // 3. Confirmar configuración aeronáutica en el hardware u-blox NEO-7M
    // Intentamos hasta 3 veces por si el módulo se encuentra completando su secuencia de arranque
    bool aeronatical_confirmed = false;
    for (int intento = 0; intento < 3; intento++) {
        if (_configureAirborneMode()) {
            aeronautical_confirmed = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // RETORNA TRUE SOLO SI LA CONFIGURACIÓN AERONÁUTICA FUE CONFIRMADA POR ACK-ACK
    if (!aeronautical_confirmed) {
        return false;
    }

    // 4. Lanzar la tarea en segundo plano para procesar NMEA y actualizar el buzón
    BaseType_t res = xTaskCreatePinnedToCore(
        _taskCallback,     // Callback estático
        "mGPS_Task",       // Nombre de la tarea
        4096,              // Tamaño de stack en words/bytes
        this,              // Puntero a esta instancia
        5,                 // Prioridad (media para no perder bytes de la FIFO de UART)
        nullptr,           // Handle no requerido
        1                  // Ejecutar en Core 1 (Core de aplicación)
    );

    return (res == pdPASS);
}

data_gps_t mGPS::get_gps_raw_data() const {
    data_gps_t pvt = {0};
    pvt.fix_type = 1;
    pvt.gnss_fix_ok = false;
    pvt.hdop = 99.9f;
    pvt.pdop = 99.9f;

    if (_gpsQueue != nullptr) {
        // xQueuePeek con timeout 0 ejecuta lectura instantánea no bloqueante.
        // No remueve el dato, permitiendo que múltiples consumidores o lecturas
        // consecutivas en el lazo de control obtengan la última trama válida al instante.
        xQueuePeek(_gpsQueue, &pvt, 0);
    }
    return pvt;
}

// --- MÉTODOS INTERNOS Y CALLBACKS ---

void mGPS::_taskCallback(void* arg) {
    mGPS* instance = static_cast<mGPS*>(arg);
    if (instance) {
        instance->_readLoop();
    }
    vTaskDelete(nullptr);
}

void mGPS::_readLoop() {
    data_gps_t current_pvt = {0};
    current_pvt.fix_type = 1;
    current_pvt.gnss_fix_ok = false;
    current_pvt.hdop = 99.9f;
    current_pvt.pdop = 99.9f;

    while (true) {
        bool sentence_parsed = false;

        // Leer todo el buffer disponible en el UART de hardware
        while (_serial->available() > 0) {
            char c = _serial->read();
            if (_gps.encode(c)) {
                sentence_parsed = true;
            }
        }

        // Si se procesó y validó una nueva sentencia NMEA (checksum correcto)
        if (sentence_parsed) {
            // 1. Resolver Fix Type desde GNGSA o GPGSA
            uint8_t fix_val = 1;
            if (_fix_gngsa.isValid() && _fix_gngsa.isUpdated()) {
                fix_val = (uint8_t)atoi(_fix_gngsa.value());
            } else if (_fix_gpgsa.isValid() && _fix_gpgsa.isUpdated()) {
                fix_val = (uint8_t)atoi(_fix_gpgsa.value());
            } else if (_fix_gngsa.isValid()) {
                fix_val = (uint8_t)atoi(_fix_gngsa.value());
            } else if (_fix_gpgsa.isValid()) {
                fix_val = (uint8_t)atoi(_fix_gpgsa.value());
            }

            // Si la localización o el tiempo fallan, degradar el fix a 1
            if (!_gps.location.isValid() || _gps.satellites.value() == 0) {
                fix_val = 1;
            }

            // 2. Resolver PDOP
            float pdop_val = 99.9f;
            if (_pdop_gngsa.isValid()) {
                pdop_val = (float)atof(_pdop_gngsa.value());
            } else if (_pdop_gpgsa.isValid()) {
                pdop_val = (float)atof(_pdop_gpgsa.value());
            }

            // 3. Poblar estructura PVT
            current_pvt.latitude    = _gps.location.lat();
            current_pvt.longitude   = _gps.location.lng();
            current_pvt.altitude    = _gps.altitude.meters();
            current_pvt.speed       = _gps.speed.mps();
            current_pvt.course      = _gps.course.deg();
            current_pvt.hdop        = (float)_gps.hdop.hdop(); // Método nativo que devuelve float (dividiendo el int por 100)
            current_pvt.pdop        = pdop_val;
            current_pvt.fix_type    = fix_val;
            current_pvt.satellites  = _gps.satellites.value();
            current_pvt.last_update = millis();

            // 4. Validación Aeronáutica Estricta para Lanzamiento
            // Se requiere Fix 3D real, HDOP aceptable (< 2.0), PDOP aceptable (<= 2.0) y mínimo 4 satélites
            current_pvt.gnss_fix_ok = (_gps.location.isValid() &&
                                   current_pvt.fix_type == 3 &&
                                   current_pvt.hdop < 2.0f &&
                                   current_pvt.pdop <= 2.0f &&
                                   current_pvt.satellites >= 4);

            // 5. Actualizar buzón de forma atómica y libre de bloqueos
            xQueueOverwrite(_gpsQueue, &current_pvt);
        }

        // Evitar inanición del procesador (Sleep 10ms -> frecuencia de chequeo ~100Hz)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool mGPS::_configureAirborneMode() {
    // Configuración UBX-CFG-NAV5 (0x06 0x24) para u-blox NEO-7M
    // Payload de 36 bytes. Configuramos la máscara para alterar únicamente el "Dynamic Platform Model"
    uint8_t payload[36] = {0};
    payload[0] = 0x01; // mask LSB: bit 0 = 1 -> Aplicar cambio en dynModel
    payload[1] = 0x00; // mask MSB
    payload[2] = 0x08; // dynModel = 8 -> "Airborne with <4g acceleration" (Ideal para cohetería)
    // El resto de bytes en 0 respetan los parámetros de vuelo por defecto al estar enmascarados

    return _sendUBXAndExpectACK(0x06, 0x24, payload, sizeof(payload), 1500);
}

bool mGPS::_sendUBXAndExpectACK(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t len, uint32_t timeoutMs) {
    // Limpiar el buffer de recepción para eliminar tramas NMEA en vuelo
    while (_serial->available() > 0) {
        _serial->read();
    }

    uint8_t ckA = 0, ckB = 0;
    auto addCheck = [&ckA, &ckB](uint8_t b) {
        ckA += b;
        ckB += ckA;
    };

    // Calcular Checksum sobre Class, ID, Length y Payload
    addCheck(msgClass);
    addCheck(msgId);
    addCheck(len & 0xFF);
    addCheck((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        addCheck(payload[i]);
    }

    // Enviar cabecera UBX y cuerpo
    _serial->write(0xB5);
    _serial->write(0x62);
    _serial->write(msgClass);
    _serial->write(msgId);
    _serial->write(len & 0xFF);
    _serial->write((len >> 8) & 0xFF);
    _serial->write(payload, len);
    _serial->write(ckA);
    _serial->write(ckB);
    _serial->flush();

    // Máquina de estados para capturar y validar paquete UBX-ACK-ACK o UBX-ACK-NAK
    uint32_t startMillis = millis();
    uint8_t step = 0;
    uint8_t ackClass = 0, ackId = 0;

    while ((millis() - startMillis) < timeoutMs) {
        if (_serial->available() > 0) {
            uint8_t b = _serial->read();
            switch (step) {
                case 0: if (b == 0xB5) step++; else step = 0; break; // Sync 1
                case 1: if (b == 0x62) step++; else step = 0; break; // Sync 2
                case 2: if (b == 0x05) step++; else step = 0; break; // Class: ACK (0x05)
                case 3:
                    if (b == 0x01) step++;       // ID: ACK-ACK (0x01) -> Aceptado
                    else if (b == 0x00) return false; // ID: ACK-NAK (0x00) -> Rechazado por u-blox
                    else step = 0;
                    break;
                case 4: if (b == 0x02) step++; else step = 0; break; // Length LSB (2 bytes)
                case 5: if (b == 0x00) step++; else step = 0; break; // Length MSB
                case 6: ackClass = b; step++; break;                 // Payload: Clase confirmada
                case 7: ackId = b; step++; break;                    // Payload: ID confirmado
                case 8: step++; break;                               // CK_A
                case 9:                                              // CK_B
                    if (ackClass == msgClass && ackId == msgId) {
                        return true; // Configuración Aeronáutica confirmada al 100% por el módulo
                    }
                    step = 0;
                    break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5)); // Evitar bloqueo de CPU mientras se espera el UART
        }
    }
    return false; // Expiró el tiempo de espera sin confirmación
}