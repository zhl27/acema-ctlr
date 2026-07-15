//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_CAMERA_H
#define ACEMA_CTLR_CAMERA_H


#include <WString.h>

#include "eEsp32Cam.h"

class Camera {
private:
    static eEsp32Cam* _espCam;
    static bool _grabando;

    // Constructor privado para evitar instanciación
    Camera() = delete;

public:
    static bool init(eEsp32Cam& espCam) {
        // if (espCam == nullptr) return false; // el ampersand evita que se pasen nullptrs
        _espCam = &espCam;
        _grabando = false;
        return true;
    }

    // Nombres declarativos optimizados
    static bool iniciarGrabacion();

    // Debe dispararse exactamente al detectar el apogeo en la fase balística
    static bool detenerGrabacion();

    static bool estaGrabando() {
        return _grabando;
    }
};


#endif //ACEMA_CTLR_CAMERA_H
