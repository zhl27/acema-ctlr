//
// Created by lucaz on 11/7/2026.
//

#include "mSD.h"

// Lista de inicialización: pasamos los parámetros al objeto _cam
mSD::mSD(int rxPin, int txPin, uint32_t baudRate) : _cam(rxPin, txPin, baudRate) {
}

bool mSD::init() {
    _cam.begin();

    // Solicitamos a la ESP32-S3-CAM que verifique si su tarjeta SD está montada
    String response = _cam.executeCommand("CMD:INIT");
    return (response.equals("ACK"));
}

bool mSD::write(const String& filename, const String& data) {
    String command = "CMD:WRITE|" + filename + "|" + data;
    String response = _cam.executeCommand(command);

    // Si la ESP32-S3-CAM guardó el archivo exitosamente, responderá con ACK
    return (response.equals("ACK"));
}

String mSD::read(const String& filename) {
    String command = "CMD:READ|" + filename;
    String response = _cam.executeCommand(command);

    // Si la respuesta comienza con el prefijo "DATA:", extraemos la información
    if (response.startsWith("DATA:")) {
        return response.substring(5);
    }

    // Si hubo error ("ERR:FILE_NOT_FOUND", timeout, etc.), retornamos vacío
    return "";
}