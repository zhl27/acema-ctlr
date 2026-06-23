#ifndef EXAMPLE-UBX-CONFIGURATOR

#include "UbxDispatcher.h"
#include "UbxConfigurator.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <HardwareSerial.h>
//#include "arduino.h"

// 1. Instancias globales para el Dispatcher

SemaphoreHandle_t ackSemaphore = xSemaphoreCreateBinary();
uint8_t expectedClass, expectedId;

// Callback que el Dispatcher ejecutará al recibir un NAV-PVT legítimo
void onPvtReceived(void* data) {
    // Recuperación del tipo de dato original de forma segura
    nav_pvt_t* pvt = reinterpret_cast<nav_pvt_t*>(data);
    
    Serial.printf("Latitud: %d\n", pvt->lat);
    // Podrías enviar este struct a una cola de FreeRTOS para otra tarea
}

// Callback para cuando llega cualquier ACK/NAK
void onAckReceived(void* data) {
    // Verificamos si es el ACK que estamos esperando
    // (Lógica simple: podrías extraer el ID del ACK recibido del buffer del dispatcher)
    xSemaphoreGive(ackSemaphore);
}


// 2. Funciones de pegamento (Glue Functions) para el Configurator
void uartTx(const uint8_t* data, size_t len) {
    uart_write_bytes(UART_NUM_1, (const char*)data, len);
}

bool waitAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
    // Almacenas qué cls/id esperas
    expectedClass = cls;
    expectedId = id;

    // Te bloqueas acá esperando que el callback de tu UbxDispatcher.cpp
    // destrabe este semáforo cuando reciba el ACK correcto.
    if (xSemaphoreTake(ackSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        return true; // Configuración aceptada
    }
    return false; // Tiempo expirado o NAK recibido
}

// 3. Tarea principal de procesamiento
void gps_task(void* pvParameters) {
    nav_pvt_t mi_pvt_data;
    // Tabla de registros que el UbxDispatcher usará para rutear
    const UbxRegMsg_t regPvt = {UBX_CLASS::NAV, UBX_ID_NAV::PVT, (uint8_t*)&mi_pvt_data, sizeof(nav_pvt_t), onPvtReceived};
    const UbxRegMsg_t regAck = {UBX_CLASS::ACK, 0x01, nullptr, 0, onAckReceived}; // ACK-ACK

    const UbxRegMsg_t* tablaRegistros[] = {&regPvt, &regAck};
    
    // Instantación
    UbxDispatcher dispatcher(tablaRegistros, 2);
    UbxConfigurator configurator(uartTx, waitAck);

    // Inicialización
    configurator.setPortUart(115200);
    configurator.setDynamicModel(NAV5_DYN_MODEL::Airborne_4G);
    configurator.enableRegisteredMessages(tablaRegistros, 2);

    uint8_t buffer[128];
    for(;;){
        int len = uart_read_bytes(UART_NUM_1, buffer, 128, pdMS_TO_TICKS(10));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                dispatcher.handleFSM(buffer[i]);
            }
        }
    }
}


#endif