//
// Created by lucaz on 9/7/2026.
//

#ifndef ACEMA_CTLR_MAIN_H
#define ACEMA_CTLR_MAIN_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"

#include <cstring>
#include <cstdio>

#include "LoraWrapped.h"
#include "core/mde_cohete/mde_cohete.h"
#include "SerialPrint.h"
#include "data.h"
#include "mBuzzer.h"
#include "services/DataFilter.h"
#include "services/GSE.h"
#include "services/Sensors.h"
#include "config.h"
#include "services/EmaFilter.h"


// constexpr size_t RBUF_SIZE = 8192; // bytes per ring buffer
constexpr size_t RBUF_SIZE = 4096; // bytes per ring buffer

// Ring buffer handles
RingbufHandle_t xDataFilterRingbuf;
RingbufHandle_t xStateMachineRingbuf;
RingbufHandle_t xLoraRingbuf;
RingbufHandle_t xFlashRingbuf;

// Task Function Prototypes
void vTaskReadSensors(void *pvParameters); // la tarea que lee los sensores y envía los datos a la cola de datos crudos
void vTaskDataFilter(void *pvParameters); // agarra los datos crudos de los sensores, los procesa y los distribuye
void vTaskStateMachine(void *pvParameters); // la máquina de estados que orquesta la lógica principal del cohete, incluyendo la gestión de estados de conexión, envío de telemetría, etc.
void vTaskFlash(void *pvParameters); // la caja negra que persiste cada dato entrante.
void vTaskLora(void *pvParameters); // maneja la comunicación LoRa, incluyendo el envío de datos y la gestión de la conexión con el GSE.

static const char *TAG_TASK_SENSORS = "TASK SENSORS";
static const char *TAG_TASK_DATA_FILTER = "TASK DATA FILTER";
static const char *TAG_TASK_STATE_MACHINE = "TASK STATE MACHINE";
static const char *TAG_TASK_FLASH = "TASK FLASH";
static const char *TAG_TASK_LORA = "TASK LORA";

mBuzzer buzzer(BUZZER_PIN);

int contadorMde = 0;
int contadorFlash = 0;
int contadorSensores = 0;
int contadorLora = 0;

// int muestreo_datos_crudos_ms = 500; // cada 0,5 segundos



#endif //ACEMA_CTLR_MAIN_H
