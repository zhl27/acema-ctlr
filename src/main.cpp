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
    initSerialLog(); // Utiliza tu función de inicializacion.cpp en lugar de duplicar código
    
    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "🚀 BOOT SEQUENCE INICIADA");
    ESP_LOGI(TAG_MAIN, "===========================================");

    // 2. Recuperación de memoria y configuración (Caja Negra)
   /* if (initBlackBox()) {
        ESP_LOGI(TAG_MAIN, "Configuración previa recuperada de la Flash.");
    } else {
        ESP_LOGW(TAG_MAIN, "No se pudo recuperar la configuración. Cargando defaults.");
    }*/

    // 3. Inicialización de Hardware Base (I2C, SPI, Sensores sin calibrar)
    bool hardIniciado = initHardware();
    if (hardIniciado) {
        ESP_LOGI(TAG_MAIN, "Hardware base inicializado.");
    } else {
        ESP_LOGE(TAG_MAIN, "CRITICAL ERROR: Fallo en initHardware(). Posible fallo en I2C.");
        //while(true) { vTaskDelay(100); } // Bucle infinito de seguridad
    }
    Actuators::getBuzzer().playError();

    // 4. Lógica de Boot y Diagnóstico de Vuelo (Decide estado y si calibra o no)
    //cpuntoDeInicio(hardIniciado); 

    // 5. Inicialización de Comunicaciones RF y Comandos
    initLora();
    if (registrarComandos()) {
        ESP_LOGI(TAG_MAIN, "Comandos de GSE registrados exitosamente.");
    } else {
        ESP_LOGW(TAG_MAIN, "Atención: Fallo al registrar algunos comandos.");
    }

    // 6. Creación de Colas y RingBuffers de FreeRTOS
    ESP_LOGI(TAG_MAIN, "Asignando memoria para Buffers de FreeRTOS...");
    xColaSensores = xQueueCreate(BUF_Q_SENSOR_SIZE, sizeof(data_raw_t));
    if (xColaSensores == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xColaSensores");

    xStateMachineRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xStateMachineRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xStateMachineRingbuf");

    xFlashRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xStateMachineRingbuf == NULL) ESP_LOGE(TAG_MAIN, "Error crítico: No se pudo crear xFlashRingbuf");
    
    // 7. Lanzamiento de las Tareas (Threads)
    ESP_LOGI(TAG_MAIN, "Desplegando Tareas de FreeRTOS en Cores...");
    
    // // Core 1: Operaciones críticas
    xTaskCreatePinnedToCore(vTaskStateMachine, "StateMachine", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskStateMachineHandle), 1);
    xTaskCreatePinnedToCore(vTaskFlash, "BlackBox", 8192, &cajaNegra, 4, &(Cohete::SYSTEM.procesos.xTaskFlashHandle), 1); 
    xTaskCreatePinnedToCore(vTaskReadSensors, "ReadSensors", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskReadSensorsHandle), 1);
    xTaskCreatePinnedToCore(vTaskDataFilter, "DataFilter", 8192, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskDataFilterHandle), 1);


    // Core 0: Operaciones de comunicación
    //xTaskCreatePinnedToCore(vTaskLora, "Lora", 8192, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskLoraHandle), 0);
    // xTaskCreatePinnedToCore(vTaskFlash, "Flash", 4096, NULL, 4, &(SYSTEM.procesos.xTaskFlashHandle), 0);

    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "🚀 BOOT SEQUENCE COMPLETADA. Entregando control a FreeRTOS.");
    ESP_LOGI(TAG_MAIN, "===========================================");

    Serial.println("off sensor");
    //Actuators::getBuzzer().off();

    // 8. Borrar la tarea "setup/loop" predeterminada
    vTaskDelete(NULL); 
}

void loop() {
    //Actuators::getBuzzer().runBuzzer();
}