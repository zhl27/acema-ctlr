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
#include <cmath> // Para atan2
#include "services/Kalman1D.h"
#include "services/Kalman2D.h"
#include "services/CmdDispatcher/CmdDispatcher.h"
#include "services/EnlaceGSE/EnlaceGSE.h"
#include "mPyro.h"

// TODO: CAMBIAR LOS PINES POR LOS REALES
#define PIN_PYRO 10
#define PIN_CONTINUIDAD_PIRO 11
#define UMBRAL_MIN_CONTINUIDAD_PYRO_mV 800

// Instancias globales de los filtros (Ajustar las varianzas empíricamente. Ej: Gyro=0.001, Accel=0.01)
Kalman1D kalmanPitch(0.001f, 0.01f);
Kalman1D kalmanYaw(0.001f, 0.01f);

// constexpr size_t RBUF_SIZE = 8192; // bytes per ring buffer
constexpr size_t RBUF_SIZE = 4096; // bytes per ring buffer
constexpr size_t BUF_Q_SENSOR_SIZE = 1024;
constexpr float PERIOD_SAMPLIG_SENSORS_MS = 10; // muestreo cada 10 ms
constexpr float FREC_SAMPLING_SENSORS_HZ = 1.0f/(float) (PERIOD_SAMPLIG_SENSORS_MS * 1e-3);

// Ring buffer handles
RingbufHandle_t xStateMachineRingbuf;
RingbufHandle_t xLoraRingbuf;
RingbufHandle_t xFlashRingbuf;

// Cola simple 
QueueHandle_t xColaSensores;

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
mPyro pirotecnico(PIN_PYRO,PIN_CONTINUIDAD_PIRO, PIN_CONTINUIDAD_PIRO);

int contadorMde = 0;
int contadorFlash = 0;
int contadorSensores = 0;
int contadorLora = 0;


CmdDispatcher cmdDispatcher;
// int muestreo_datos_crudos_ms = 500; // cada 0,5 segundos

constexpr float R_AIR = 287.05f;

inline float calcularDensidadAire(const float pressure_hpa, const float temperature_deg_c)
{
    return (pressure_hpa * 100.0f) / (R_AIR * (temperature_deg_c + 273.15f));
}

#endif //ACEMA_CTLR_MAIN_H
