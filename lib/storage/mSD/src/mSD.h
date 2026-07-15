//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_MSD_H
#define ACEMA_CTLR_MSD_H

#include <Arduino.h>
#include "eEsp32Cam.h"

class mSD {
private:
    eEsp32Cam* _espCam;
    bool _inicializado = false;

public:
    explicit mSD(eEsp32Cam& espCam) : _espCam(&espCam) {}
    // int rxPin = 34, int txPin = 26, uint32_t baudRate = 115200

    bool init();

    // Escribe datos (ej: línea CSV o buffer de flash) en un archivo de la SD
    bool escribir(const char* rutaArchivo, const char* datos) const;

    // Lee el contenido de un archivo (útil para verificar configuraciones de tierra)
    // TODO: FALTA IMPLEMENTAR mSD::leer, para futuros reqs
    // bool leer(const char* rutaArchivo, String& bufferSalida) const;
};


#endif //ACEMA_CTLR_MSD_H
