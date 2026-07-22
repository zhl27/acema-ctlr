/**
 * @file main.cpp
 * @brief Punto de entrada principal del Firmware
 * @author Joe Cruz
 * @date 19-jul-2026
 */

#include <Arduino.h>
#include <esp_log.h>

#include "main.h"
#include "config.h"
#include "inicializacion.h" 
#include "LoraWrapped.h"

// Tags para logs
static const char *TAG_MAIN = "MAIN_SETUP";

using namespace ConfigInit;

void setup() {
    // 1. Inicialización de la consola y logs
    Serial.begin(SERIAL_BAUDRATE_LOG); 
    while (!Serial) { delay(100); }

    // Por defecto ESP-IDF loguea en INFO, si querés DEBUG descomenta:
    // esp_log_level_set("*", ESP_LOG_DEBUG);

    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "🚀 BOOT SEQUENCE INICIADA");
    ESP_LOGI(TAG_MAIN, "===========================================");

    // 2. Inicialización del Hardware y Sensores
    if (initHardware()) {
        ESP_LOGI(TAG_MAIN, "Hardware inicializado correctamente.");
    } else {
        ESP_LOGE(TAG_MAIN, "CRITICAL ERROR: Fallo en initHardware(). Posible fallo en I2C o sensores.");
        // Podrías poner un bucle infinito aquí si el cohete no debe volar sin sensores
    }

    // 3. Registro de Comandos Bidireccionales
    if (registrarComandos()) {
        ESP_LOGI(TAG_MAIN, "Comandos de GSE registrados exitosamente.");
    } else {
        ESP_LOGW(TAG_MAIN, "Atención: Fallo al registrar algunos comandos.");
    }

    // 4. Creación de Colas y RingBuffers
    ESP_LOGI(TAG_MAIN, "Asignando memoria para Buffers de FreeRTOS...");

    xColaSensores = xQueueCreate(BUF_Q_SENSOR_SIZE, sizeof(data_raw_t));
    if (xColaSensores == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xColaSensores");

    xStateMachineRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xStateMachineRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xStateMachineRingbuf");

    xLoraRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xLoraRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xLoraRingbuf");

    // xFlashRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    // if (xFlashRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xFlashRingbuf");

    // 5. Inicializar la capa de enlace GSE
    EnlaceGSE::inicializar(xLoraRingbuf);
    ESP_LOGI(TAG_MAIN, "Enlace LoRa/GSE configurado.");

    // 6. Lanzamiento de las Tareas (Threads)
    ESP_LOGI(TAG_MAIN, "Desplegando Tareas de FreeRTOS en Cores...");
    
    // Core 1: Operaciones críticas de lectura y filtrado
    xTaskCreatePinnedToCore(vTaskReadSensors, "ReadSensors", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskReadSensorsHandle), 1);
    xTaskCreatePinnedToCore(vTaskDataFilter, "DataFilter", 8192, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskDataFilterHandle), 1);
    xTaskCreatePinnedToCore(vTaskStateMachine, "StateMachine", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskStateMachineHandle), 1);

    // Core 0: Operaciones de comunicación y persistencia (WiFi/LoRa/Flash viven mejor acá)
    // xTaskCreatePinnedToCore(vTaskFlash, "Flash", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskFlashHandle), 0);
    xTaskCreatePinnedToCore(vTaskLora, "Lora", 8192, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskLoraHandle), 0);

    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "🚀 BOOT SEQUENCE COMPLETADA. Entregando control a FreeRTOS.");
    ESP_LOGI(TAG_MAIN, "===========================================");

    // 7. Borrar la tarea "setup/loop" predeterminada de Arduino para liberar RAM
    vTaskDelete(NULL); 
}

void loop() {
    // Intencionalmente vacío. Borrado en setup mediante vTaskDelete.
}