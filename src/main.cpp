#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#include <cstring>
#include <cstdio>


#include "LoraWrapped.h"
#include "core/mde_cohete/mde_cohete.h"
#include "SerialPrint.h"
#include "data.h"


// Pines asignados si compilas con: pio run -e CPU-esp32
    #define LORA_SCK  18
    #define LORA_MISO 19
    #define LORA_MOSI 23
    #define LORA_CS   5
    #define LORA_RST  14
    #define LORA_DIO0 2
    #define LORA_DIO1 4
// Instanciación única y genérica usando los alias de los macros
LoraWrapped lora(LORA_CS, LORA_RST, LORA_DIO0, LORA_DIO1, SPI);

constexpr size_t RBUF_SIZE = 1024; // bytes per ring buffer

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
        delay(1000); // will pause mcu until serial console opens

    SerialPrint::msg("Setup");


    // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
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

    // Create tasks with priority hierarchy
    xTaskCreate(vTaskReadSensors, "ReadSensors", 4096, NULL, 4, &xTaskReadSensorsHandle); // TODO: Hacer un Profile. Ver si es overkill usar 4096 WORDs para esto. Ojo: WORD = 4 bits en la esp32. "You can use uxTaskGetStackHighWaterMark() to monitor unused stack space"
    xTaskCreate(vTaskStateMachine, "StateMachine", 4096, NULL, 3, &xTaskStateMachineHandle);
    xTaskCreate(vTaskFlash, "Flash", 4096, NULL, 3, &xTaskFlashHandle);
    xTaskCreate(vTaskLora, "Lora", 4096, NULL, 2, &xTaskLoraHandle);

    vTaskDelete(NULL); // NULL hace referenica al task default que maneja a "void loop()"

    delay(100);
}

void loop(){
    // NO SE USA ESTO
}

// TODO: Pensar sobre este texto: "You need to gather large bursts of hardware data inside an Interrupt Service Routine (ISR) to be processed later by a task."
void vTaskReadSensors(void *pvParameters) {
    while (1) {
        // Mock: generate a fake sensor payload and send to the ring buffers
        char payload[128];
        static int seq = 0;
        float mock_alt = 100.0f + (seq * 0.1f);
        float mock_acc = 0.01f * seq;
        int len = snprintf(
            payload,
            sizeof(payload),
            "SENSOR;seq=%d;alt=%.2f;acc=%.3f", seq++, mock_alt, mock_acc);

        Serial.printf("[ReadSensors] Emitting: %s\n", payload);

        if (xStateMachineRingbuf != NULL) {
            if (xRingbufferSend(xStateMachineRingbuf, (void *)payload, (size_t)(len + 1), pdMS_TO_TICKS(10)) != pdTRUE) {
                SerialPrint::err("xRingbufferSend -> xStateMachineRingbuf failed");
            }
        }

        if (xFlashRingbuf != NULL) {
            if (xRingbufferSend(xFlashRingbuf, (void *)payload, (size_t)(len + 1), pdMS_TO_TICKS(10)) != pdTRUE) {
                SerialPrint::err("xRingbufferSend -> xFlashRingbuf failed");
            }
        }

        if (xLoraRingbuf != NULL) {
            if (xRingbufferSend(xLoraRingbuf, (void *)payload, (size_t)(len + 1), pdMS_TO_TICKS(10)) != pdTRUE) {
                SerialPrint::err("xRingbufferSend -> xLoraRingbuf failed");
            }
        }

        // Simulate a 1 second sampling interval
        vTaskDelay(pdMS_TO_TICKS(1000)); // it yields CPU to lower priorities for 1s
    }
}

// Mock implementation of the State Machine task: consumes sensor messages and forwards/acts on them
void vTaskStateMachine(void *pvParameters) {
    (void)pvParameters;
    for (;;) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (StateMachine)", xPortGetCoreID());
#endif

        size_t item_size = 0;
        const auto item = static_cast<char *>(xRingbufferReceive(xStateMachineRingbuf, &item_size, pdMS_TO_TICKS(2000)));
        if (item != nullptr) {
            // 2. Validar que el tamaño recibido coincide exactamente con nuestro struct
            if (item_size == sizeof(data_all_t)) {

                auto* datos_sensores = reinterpret_cast<data_all_t *>(item);

                // (Opcional) Guardar una copia por si hay que evaluar la MDE sin datos nuevos
                // memcpy(&ultimos_datos, datos_sensores, sizeof(data_all_t));

                mde_cohete_actualizar(datos_sensores);

            } else {
                SerialPrint::msg("[StateMachine] ERROR: Tamaño de item no coincide con data_all_t");
            }

            // Liberar la memoria del RingBuffer para que el productor pueda seguir escribiendo
            vRingbufferReturnItem(xStateMachineRingbuf, item);

        } else {
            // TIMEOUT: No llegaron datos nuevos en los últimos 10ms.
            // Si la MDE necesita evaluar temporizadores (ej. ST_EVALUAR_SUPERVIVENCIA_DROGUE)
            // podrías llamarla aquí pasándole 'ultimos_datos'.
            SerialPrint::msg("[StateMachine] No messages (timeout)");
        }
    }
}

// Mock implementation of the Flash task: consumes items from the Flash ringbuffer and "persists" them
void vTaskFlash(void *pvParameters) {
    (void)pvParameters;
    for (;;) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (Flash)", xPortGetCoreID());
#endif

        size_t item_size = 0;
        char *item = (char *) xRingbufferReceive(xFlashRingbuf, &item_size, pdMS_TO_TICKS(5000));
        if (item != NULL) {
            Serial.printf("[Flash] Persisting (%d bytes): %s\n", (int)item_size, item);
            // TODO: In a real implementation, write to SD/flash. Here we just log.
            vRingbufferReturnItem(xFlashRingbuf, (void *)item);
        } else {
            SerialPrint::msg("[Flash] No items to persist (timeout)");
        }
    }
}

// Mock implementation of the Lora task: consumes items and "sends" them over LoRa
void vTaskLora(void *pvParameters) {
    (void)pvParameters;
    for (;;) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (Lora)", xPortGetCoreID());
#endif

        size_t item_size = 0;
        char *item = (char *) xRingbufferReceive(xLoraRingbuf, &item_size, pdMS_TO_TICKS(3000));
        if (item != NULL) {
            Serial.printf("[Lora] Sending (%d bytes): %s\n", (int)item_size, item);
            // TODO: In a real implementation, pass to the LoraWrapped instance. Here we just log.
            vRingbufferReturnItem(xLoraRingbuf, (void *)item);
        } else {
            SerialPrint::msg("[Lora] No messages to send (timeout)");
        }
    }
}