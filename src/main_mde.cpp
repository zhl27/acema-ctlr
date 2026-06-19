#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#include "SerialPrint.h"
#include "globals.h"
#include "data.h"
#include "sensores.h"

// Global instance of the flight system // TODO: PENSAR SI ES NECESARIO
// sistema_vuelo_t SISTEMA = {
//     .sensores = {
//         .altura = 0.0f,
//         .inclinacion = 0.0f,
//         .aceleracion = 0.0f,
//         .vel_vertical = 0.0f,
//         .variacion_aceleracion = 0.0f,
//         .estado_hardware = HARDWARE_OK
//     },
//     .estado_vuelo = ST_INIT,
//     .codigo_error = 0
// };

const size_t RBUF_SIZE = 1024; // bytes per ring buffer

// Ring buffer handles
RingbufHandle_t xStateMachineRingbuf;
RingbufHandle_t xLoraRingbuf;
RingbufHandle_t xFlashRingbuf;

// Task Handles
TaskHandle_t xTaskReadSensorsHandle = NULL;
TaskHandle_t xTaskStateMachineHandle = NULL;
TaskHandle_t xTaskFlashHandle = NULL;
TaskHandle_t xTaskLoraHandle = NULL;

// Task Function Prototypes
void vTaskReadSensors(void *pvParameters); // la tarea que lee los sensores y envía los datos a la cola --> Productor
void vTaskStateMachine(void *pvParameters); // la máquina de estados que orquesta la lógica principal del cohete, incluyendo la gestión de estados de conexión, envío de telemetría, etc.
void vTaskFlash(void *pvParameters); // la caja negra que persiste cada dato entrante.
void vTaskLora(void *pvParameters); // maneja la comunicación LoRa, incluyendo el envío de datos y la gestión de la conexión con el GSE.


void setup() {
    Serial.begin(115200);

    while (!Serial)
        delay(10); // will pause mcu until serial console opens

    SerialPrint::msg("Setup");


    xStateMachineRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xStateMachineRingbuf == NULL) {
        SerialPrint::err("Error al crear xStateMachineRingbuf");
    } else {
        SerialPrint::msg("xStateMachineRingbuf creado");
    }

    xLoraRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xLoraRingbuf == NULL) {
        SerialPrint::err("Error al crear xLoraRingbuf");
    } else {
        SerialPrint::msg("xLoraRingbuf creado");
    }

    xFlashRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xFlashRingbuf == NULL) {
        SerialPrint::err("Error al crear xFlashRingbuf");
    } else {
        SerialPrint::msg("xFlashRingbuf creado");
    }

    vTaskDelete(NULL); // NULL hace referenica al task default que maneja a "void loop()"

    delay(100);
}


// TODO: Pensar sobre este texto: "You need to gather large bursts of hardware data inside an Interrupt Service Routine (ISR) to be processed later by a task."
void vTaskReadSensors(void *pvParameters) {
    while (1) {
        SerialPrint::msg("Leyendo sensores...");
        // Aquí iría la lógica de lectura de sensores y envío a la cola
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Simula un retardo de 1 segundo entre lecturas
    }
}