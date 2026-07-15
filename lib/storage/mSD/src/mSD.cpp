//
// Created by lucaz on 11/7/2026.
//

#include "mSD.h"


bool mSD::init() {
    if (_espCam == nullptr) return false;

    // Verificamos si la tarjeta SD en la ESP-CAM está montada y lista
    String respuesta;
    _inicializado = _espCam->ejecutar_comando_sincrono("SD_INIT", "", respuesta);
    return _inicializado;
}

bool mSD::escribir(const char* rutaArchivo, const char* datos) const {
    if (!_inicializado || _espCam == nullptr) return false;

    // Empaquetamos ruta y datos: "/vuelo.csv:1023,9.81,500"
    const String payload = String(rutaArchivo) + ":" + String(datos);
    String respuesta;

    return _espCam->sendCommandToCam(CMD_SD_DUMP_START, payload, respuesta); // TODO: REVISAR
}

// bool mSD::leer(const char* rutaArchivo, String& bufferSalida) const {
//     if (!_inicializado || _espCam == nullptr) return false;
//
//     String respuesta;
//     if (_espCam->ejecutar_comando_sincrono("SD_RD", String(rutaArchivo), bufferSalida)) {
//         // Limpiamos la cabecera del protocolo de la respuesta
//         const int idx = bufferSalida.indexOf(',');
//         if (idx != -1) {
//             bufferSalida = bufferSalida.substring(idx + 1);
//         }
//         return true;
//     }
//     return false;
// }

