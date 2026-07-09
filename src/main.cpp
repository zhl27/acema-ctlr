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

#include <SPI.h>
#include "LoraWrapped.h"


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

static const char *TAG_SENSORS = "SENSORS";
static const char *TAG_DATA_FILTER = "DATA FILTER";
static const char *TAG_STATE_MACHINE = "STATE MACHINE";
static const char *TAG_FLASH = "FLASH";
static const char *TAG_LORA = "LORA";

mBuzzer buzzer(BUZZER_PIN);

int contadorMde = 0;
int contadorFlash = 0;
int contadorSensores = 0;
int contadorLora = 0;

// int muestreo_datos_crudos_ms = 500; // cada 0,5 segundos

void setup() {
    Serial.begin(115200); // TODO: Para la Compu de vuelo no se usa Serial

    while (!Serial)
        delay(1000);

    ESP_LOGI("SETUP", "Comenzando SETUP.");
#ifdef DEBUG_ESP32
    esp_log_level_set("*", ESP_LOG_DEBUG);
#endif
    // buzzer.init();
    // buzzer.beep(500);

    // Initialize the kinematic filter (Adjust mass and pad offset as needed for your launch)
    // TODO: FALTA MODIFICAR DATAFILTER DE FORMA ACORDE A LOS REQUERIMIENTOS.
    DataFilter::init();
    Sensors::init();
    GSE::init();

    // DATA DISTRIBUTOR
    xDataFilterRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xDataFilterRingbuf == NULL) {
        SerialPrint::err("Error al crear xDataDistributorRingbuf");
    } else {
        SerialPrint::msg("xDataDistributorRingbuf creado");
    }

    // MDE
    // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
    xStateMachineRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xStateMachineRingbuf == NULL) {
        SerialPrint::err("Error al crear xStateMachineRingbuf");
    } else {
        SerialPrint::msg("xStateMachineRingbuf creado");
    }

    // LORA
    xLoraRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xLoraRingbuf == NULL) {
        SerialPrint::err("Error al crear xLoraRingbuf");
    } else {
        SerialPrint::msg("xLoraRingbuf creado");
    }

    // FLASH
    // xFlashRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    // if (xFlashRingbuf == NULL) {
    //     SerialPrint::err("Error al crear xFlashRingbuf");
    // } else {
    //     SerialPrint::msg("xFlashRingbuf creado");
    // }

    // TODO: Hacer un Profile. Ver si es overkill usar 4096 WORDs para esto. Ojo: WORD = 4 bits en la esp32. "You can use uxTaskGetStackHighWaterMark() to monitor unused stack space"
    xTaskCreate(vTaskReadSensors, "ReadSensors", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskReadSensorsHandle));
    xTaskCreate(vTaskStateMachine, "StateMachine", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskStateMachineHandle));
    // xTaskCreate(vTaskFlash, "Flash", 4096, NULL, 3, &xTaskFlashHandle);
    xTaskCreate(vTaskLora, "Lora", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskLoraHandle));
    xTaskCreate(vTaskDataFilter, "DataFilter", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskDataFilterHandle));

    vTaskDelete(NULL); // NULL hace referenica al task default que maneja a "void loop()"

    // buzzer.playSuccess();

    delay(100);
}

void loop(){
    // NO SE USA ESTO
}

// TODO: Pensar sobre este texto: "You need to gather large bursts of hardware data inside an Interrupt Service Routine (ISR) to be processed later by a task."
void vTaskReadSensors(void *pvParameters) {
    (void)pvParameters;
    while (true) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (ReadSensors)", xPortGetCoreID());
#endif
        // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
        data_raw_t raw = Sensors::get_raw_data();

        // print_data_raw(&raw);

        if (xDataFilterRingbuf != NULL) {
            if (xRingbufferSend(xDataFilterRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                SerialPrint::err("xRingbufferSend -> xDataDistributorRingbuf failed (raw)");
            }
        }

        // SerialPrint::plot("contadorSensores", contadorSensores);
        // contadorSensores++;

        // Simulate a 1 second sampling interval
        // vTaskDelay(pdMS_TO_TICKS(1000)); // it yields CPU to lower priorities for 1s
    }
}

void vTaskDataFilter(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        size_t item_size = 0;
        void *item = xRingbufferReceive(xDataFilterRingbuf, &item_size, pdMS_TO_TICKS(2000));

        if (item != NULL) {
            if (item_size == sizeof(data_raw_t)) {

                data_raw_t *raw_ptr = static_cast<data_raw_t *>(item);
                data_all_t all_data = DataFilter::process(*raw_ptr);
                print_data(&all_data);

                // MDE
                if (xStateMachineRingbuf != NULL) {
                    if (xRingbufferSend(xStateMachineRingbuf, (void *)&all_data, sizeof(data_all_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                        SerialPrint::err("xRingbufferSend -> xStateMachineRingbuf failed (all_data)");
                    }
                }

                // FLASH
                // if (xFlashRingbuf != NULL) {
                //     // if (xRingbufferSend(xFlashRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                //     //     SerialPrint::err("xRingbufferSend -> xFlashRingbuf failed (raw)");
                //     // }
                //     if (xRingbufferSend(xFlashRingbuf, (void *)&all_data, sizeof(data_all_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                //         SerialPrint::err("xRingbufferSend -> xFlashRingbuf failed (all_data)");
                //     }
                // }

                // LORA
                if (xLoraRingbuf != NULL && Cohete::SYSTEM.procesos.flujos.Sensors_a_Lora_enabled) {
                    // if (xRingbufferSend(xLoraRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                    //     SerialPrint::err("xRingbufferSend -> xLoraRingbuf failed (raw)");
                    // }
                    BaseType_t res = xRingbufferSend(xLoraRingbuf, (void *)&all_data, sizeof(data_all_t), pdMS_TO_TICKS(50));
                    if (res != pdTRUE) {
                        Serial.printf("xRingbufferSend (xLoraRingbuf) ha fallado (all_data). Codigo de error:%d\n", res);
                    }
                }

            } else {
                SerialPrint::err("[DataDistributor] Tamaño de item no coincide con data_raw_t");
            }
            vRingbufferReturnItem(xDataFilterRingbuf, item);
        } else {
            SerialPrint::msg("[DataDistributor] No messages (timeout)");
        }
    }
}


// Mock implementation of the State Machine task: consumes sensor messages and forwards/acts on them
void vTaskStateMachine(void *pvParameters) {
    (void)pvParameters;
    while (true) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (StateMachine)", xPortGetCoreID());
#endif

        size_t item_size = 0;
        // 1. Receive as a generic void pointer
        void *item = xRingbufferReceive(xStateMachineRingbuf, &item_size, pdMS_TO_TICKS(2000));

        if (item != NULL) {
            if (item_size == sizeof(data_all_t)) {

                data_all_t *datos_sensores = static_cast<data_all_t *>(item);

                // NOTA: Asegurarse de que mde_cohete_actualizar acepte un puntero a data_all_t
                Cohete::mde_cohete_actualizar(datos_sensores);

                // SerialPrint::plot("contadorMde", contadorMde);
                // contadorMde++;
            } else {
                SerialPrint::err("[StateMachine] Tamaño de item no coincide con data_all_t");
            }

            // 4. Free the memory
            vRingbufferReturnItem(xStateMachineRingbuf, item);

        } else {
            SerialPrint::msg("[StateMachine] No messages (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

// Mock implementation of the Flash task: consumes items from the Flash ringbuffer and "persists" them
void vTaskFlash(void *pvParameters) {
    (void)pvParameters;
    while (true) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (Flash)", xPortGetCoreID());
#endif

        size_t item_size = 0;
        void *item = xRingbufferReceive(xFlashRingbuf, &item_size, pdMS_TO_TICKS(5000));

        if (item != NULL) {
            if (item_size == sizeof(data_all_t)) {

                data_all_t *datos_sensores = static_cast<data_all_t *>(item);

                // Print a specific member of the struct (like elapsed_time) instead of %s
                // Serial.printf("[Flash] Persisting (%d bytes). Time: %lu\n", static_cast<int>(item_size), micros());
                // SerialPrint::plot("contadorFlash", contadorFlash);
                // contadorFlash++;
                // TODO: In a real implementation, write 'datos' to SD/flash.
            }
            vRingbufferReturnItem(xFlashRingbuf, item);
        } else {
            SerialPrint::msg("[Flash] No items to persist (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Mock implementation of the Lora task: consumes items and "sends" them over LoRa
void vTaskLora(void *pvParameters) {
    (void)pvParameters;
    while (true) {

#ifdef DEBUG_ESP32
        SerialPrint::plot("Core ID (Lora)", xPortGetCoreID());
#endif

        size_t item_size = 0;
        void *item = xRingbufferReceive(xLoraRingbuf, &item_size, pdMS_TO_TICKS(3000));

        if (item != NULL) {
            if (item_size == sizeof(data_all_t)) {

                data_all_t *datos_sensores = static_cast<data_all_t *>(item);

                // Print a specific member of the struct instead of %s
                // Serial.printf("[Lora] Sending (%d bytes). Time: %lu\n", static_cast<int>(item_size), micros());
                GSE::actualizar(datos_sensores);
                // SerialPrint::plot("contadorLora", contadorLora);
                // contadorLora++;
            }
            vRingbufferReturnItem(xLoraRingbuf, item);
        } else {
            SerialPrint::msg("[Lora] No messages to send (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000)); // importante ceder tiempo si hay task priorities diferentes para que no se produzca inanicion en otras tasks
    }
}