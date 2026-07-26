/**
 * @file inicializacion.h
 * @brief Declaraciones públicas (extern) de objetos globales y prototipos de tareas de FreeRTOS.
 * @author Joe Cruz
 * @date 19-jul-2026
 */

#ifndef INICIALIZACION_H
#define INICIALIZACION_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>

// ---------------------------------------------------------
// Variables y Buffers Globales (Exportados con extern)
// ---------------------------------------------------------

// Ring buffer handles (Comunicación entre tareas)
extern RingbufHandle_t xStateMachineRingbuf;
extern RingbufHandle_t xLoraRingbuf;
extern RingbufHandle_t xFlashRingbuf;

// Cola simple (Datos crudos desde lectura a filtrado)
extern QueueHandle_t xColaSensores;

// ---------------------------------------------------------
// Prototipos de Secuencias de arranque
// ---------------------------------------------------------
/**
 * @brief Inicializa el puerto serial 
 */
void initSerialLog();

/**
 * @brief Inicializa la memria flash y carga la configuración
 */
bool init_black_box();

/**
 * @brief Inicializa el hardware (sensores, actuadores, bus I2C) 
 * @return true si todo inició correctamente, false en caso de error.
 */
bool init_hardware(); // Corregí el typo de "initHarware" a "initHardware"

 /**
 * @brief Aplica la lógica de boot y decide el estado de inicio
 */
void run_boot_logic(bool);

 /**
 * @brief Inicializa el lora y canal de comunicaciones 
 */
void init_lora();

/**
 * @brief Registra los callbacks en el CmdDispatcher
 * @return true si se registraron correctamente
 */
bool registrar_comandos_gse();

// ---------------------------------------------------------
// Prototipos de Tareas de FreeRTOS
// ---------------------------------------------------------
/**
 * @brief Lee los sensores I2C/SPI y envía los datos crudos a xColaSensores
 */
void vTaskReadSensors(void *pvParameters);

/**
 * @brief Procesa los datos crudos (Kalman/EMA) y los distribuye a los RingBuffers
 */
void vTaskDataFilter(void *pvParameters);

/**
 * @brief Máquina de estados que orquesta la lógica principal del cohete
 */
void vTaskStateMachine(void *pvParameters);

/**
 * @brief Persiste la telemetría y eventos en la memoria Flash/SD
 */
void vTaskFlash(void *pvParameters);

/**
 * @brief Maneja la comunicación bidireccional LoRa con la base (GSE)
 */
void vTaskLora(void *pvParameters);

// ---------------------------------------------------------
// Prototipos de Inicialización
// ---------------------------------------------------------
extern mFlash cajaNegra;
extern mBuzzer buzzer;



#endif // INICIALIZACION_H