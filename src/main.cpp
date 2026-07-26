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

    Serial.begin(115200); // TODO: Para la Compu de vuelo no se usa Serial
    while (!Serial)
        delay(10);
    
    ESP_LOGI(TAG_MAIN, "🚀 BOOT SEQUENCE INICIADA");

    // Inicialización de Hardware Base (I2C, SPI, Sensores sin calibrar)
    const bool hw_iniciado_con_exito = init_hardware();
    if (hw_iniciado_con_exito) {
        ESP_LOGI(TAG_MAIN, "[init_hardware]: Hardware inicializado exitósamente.");
    } else {
        ESP_LOGE(TAG_MAIN, "[init_hardware]: Fallo en init_hardware(). Posible fallo en I2C.");
        //while(true) { vTaskDelay(100); } // Bucle infinito de seguridad
    }

    // Lógica de Boot y Diagnóstico de Vuelo (Decide estado y si calibra o no)
    run_boot_logic(hw_iniciado_con_exito);

    // Inicialización de Comunicaciones RF y Comandos
    init_lora();
    if (registrar_comandos_gse()) {
        ESP_LOGI(TAG_MAIN, "[registrar_comandos_gse]: Comandos de GSE registrados exitosamente.");
    } else {
        ESP_LOGW(TAG_MAIN, "[registrar_comandos_gse]: Fallo al registrar algunos comandos.");
    }

    // Recuperación de memoria y configuración (Caja Negra)
    if (init_black_box()) {
        ESP_LOGI(TAG_MAIN, "[init_black_box]: Configuración previa recuperada de la Flash.");
    } else {
        ESP_LOGW(TAG_MAIN, "[init_black_box]: No se pudo recuperar la configuración. Cargando defaults.");
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
    
    // Core 1: Operaciones críticas
    xTaskCreatePinnedToCore(vTaskStateMachine, "StateMachine", 4096, NULL, TASK_PRIORITY_COMMON, &(Cohete::SYSTEM.procesos.xTaskStateMachineHandle), 1);
    xTaskCreatePinnedToCore(vTaskFlash, "BlackBox", 8192, &cajaNegra, TASK_PRIORITY_COMMON, &(Cohete::SYSTEM.procesos.xTaskFlashHandle), 1);
    xTaskCreatePinnedToCore(vTaskReadSensors, "ReadSensors", 4096, NULL, TASK_PRIORITY_COMMON, &(Cohete::SYSTEM.procesos.xTaskReadSensorsHandle), 1);
    xTaskCreatePinnedToCore(vTaskDataFilter, "DataFilter", 8192, NULL, TASK_PRIORITY_COMMON, &(Cohete::SYSTEM.procesos.xTaskDataFilterHandle), 1);


    // Core 0: Operaciones de comunicación
    xTaskCreatePinnedToCore(vTaskLora, "Lora", 8192, NULL, TASK_PRIORITY_COMMON, &(Cohete::SYSTEM.procesos.xTaskLoraHandle), 0);

    ESP_LOGI(TAG_MAIN, "🚀 BOOT SEQUENCE COMPLETADA. Entregando control a FreeRTOS.");

    // 8. Borrar la tarea "setup/loop" predeterminada
    vTaskDelete(NULL); 
}

void loop() {
    // Vacío
}