//
// Created by lucaz on 11/7/2026.
//

#include "eEsp32Cam.h"


uint8_t eEsp32Cam::_calcularChecksum(const String& comando) {
    uint8_t checksum = 0;
    for (int i = 0; i < comando.length(); i++) {
        checksum ^= comando[i];
    }
    return checksum;
}


bool eEsp32Cam::enviar_comando(const String& cmdType, const String& payload = "") const {
    const String trama = "$" + cmdType + "," + String(payload.length()) + "," + payload;
    const uint8_t chk = _calcularChecksum(trama);

    _uart->print(trama);
    _uart->print("*");
    _uart->println(chk, HEX);
    _uart->flush();

    return true;
}


String eEsp32Cam::recibir_respuesta() const {
    unsigned long startMillis = millis();
    String respuesta = "";

    while ((millis() - startMillis) < _timeoutMs) {
        if (_uart->available()) {
            char c = _uart->read();
            if (c == '\n') {
                break; // Fin de la trama recibido
            }
            respuesta += c;
        }
    }
    return respuesta;
}


bool eEsp32Cam::ejecutar_comando_sincrono(const String& cmdType, const String& payload, String& respuestaOut) const {
    if (!enviar_comando(cmdType, payload)) return false; // TODO: el compilador dice que la condicion es siempre falsa.
    respuestaOut = recibir_respuesta();
    return respuestaOut.startsWith("$OK") || respuestaOut.startsWith("$DATA");
}
