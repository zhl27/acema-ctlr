//
// Created by lucaz on 11/7/2026.
//

#include "eEsp32Cam.h"

eEsp32Cam::eEsp32Cam(int rxPin, int txPin, uint32_t baudRate) {
    _serial = &Serial1; // Usamos el UART1 de hardware del ESP32
    _rxPin = rxPin;
    _txPin = txPin;
    _baudRate = baudRate;
    _uartMutex = xSemaphoreCreateMutex();
}

void eEsp32Cam::begin() {
    // Configuración de la matriz de enrutamiento GPIO para UART
    _serial->begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);
    clearRxBuffer();
}

void eEsp32Cam::clearRxBuffer() {
    while (_serial->available()) {
        _serial->read();
    }
}

void eEsp32Cam::sendCommand(const char* cmd) {
    clearRxBuffer();      // Evita leer respuestas viejas o ruido acumulado
    _serial->println(cmd);
    _serial->flush();     // Espera a que el último byte salte por el pin TX
}

String eEsp32Cam::receiveResponse(unsigned long timeoutMillis) {
    unsigned long startMillis = millis();
    while (millis() - startMillis < timeoutMillis) {
        if (_serial->available()) {
            String response = _serial->readStringUntil('\n');
            response.trim(); // Elimina \r y espacios en blanco
            return response;
        }
        delay(2); // Pequeña pausa para ceder tiempo a las tareas de FreeRTOS
    }
    return ""; // Cadena vacía si ocurre un timeout
}

char* eEsp32Cam::executeCommand(const char* cmd, unsigned long timeoutMillis) {
    char* response;
    // Solicitamos el "candado" de la UART. Si otra tarea la está usando, esperamos.
    if (xSemaphoreTake(_uartMutex, portMAX_DELAY) == pdTRUE) {
        sendCommand(cmd);
        response = receiveResponse(timeoutMillis);
        // Soltamos el "candado"
        xSemaphoreGive(_uartMutex);
    }
    return response;
}