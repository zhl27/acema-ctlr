//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_EESP32CAM_H
#define ACEMA_CTLR_EESP32CAM_H


// En eEsp32Cam.h
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdint.h>

class eEsp32Cam {
private:
    // ...
public:
    eEsp32Cam() {
    }

    String executeCommand(const String& cmd) {
        String response = "";
        // Solicitamos el "candado" de la UART. Si otra tarea la está usando, esperamos.
        if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
            sendCommand(cmd);
            response = receiveResponse();
            // Soltamos el "candado"
            xSemaphoreGive(uartMutex);
        }
        return response;
    }
};


#ifndef E_ESP32_CAM_H
#define E_ESP32_CAM_H

#include <Arduino.h>

class eEsp32Cam {
private:
    SemaphoreHandle_t _uartMutex; // Protege el puerto serie
    HardwareSerial* _serial;
    int _rxPin;
    int _txPin;
    uint32_t _baudRate;
    void clearRxBuffer();

public:
    // Configuración física por defecto: D34 (RX), D26 (TX)
    eEsp32Cam(int rxPin = 34, int txPin = 26, uint32_t baudRate = 115200);

    // Inicializa el bus UART
    void begin();

    // Envía un comando por TX
    void sendCommand(const char* cmd);

    // Espera y lee la respuesta por RX dentro de un tiempo máximo (timeout)
    char* receiveResponse(unsigned long timeoutMillis = 2000);

    // Método transaccional: Limpia buffer, envía comando y devuelve la respuesta
    char* executeCommand(const char* cmd, unsigned long timeoutMillis = 2000);
};

#endif // E_ESP32_CAM_H

#endif //ACEMA_CTLR_EESP32CAM_H
