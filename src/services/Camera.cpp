//
// Created by lucaz on 11/7/2026.
//

#include "Camera.h"

eEsp32Cam* Camera::_espCam = nullptr;
bool Camera::_grabando = false;

bool Camera::iniciarGrabacion() {
    if (_espCam == nullptr || _grabando) return false;

    String resp; // TODO: Usar String puede impactar a la eficiencia? Si es así, cuánto? Podríamos sacrificar eficiencia por legibilidad y mantenibilidad?
    // La ESP-CAM iniciará el guardado del flujo de video OV2640 a 25 fps en su SD
    if (_espCam->ejecutar_comando_sincrono("CAM_REC_START", "", resp)) {
        _grabando = true;
        return true;
    }
    return false;
}

bool Camera::detenerGrabacion() {
    if (_espCam == nullptr || !_grabando) return false;

    String resp;
    if (_espCam->ejecutar_comando_sincrono("CAM_REC_STOP", "", resp)) {
        _grabando = false;
        return true;
    }
    return false;
}