#include <SPI.h>
#include "LoraWrapped.h"

#include "main.h"

void setup() {
    Serial.begin(115200); // TODO: Para la Compu de vuelo no se usa Serial

    while (!Serial)
        delay(1000);

    ESP_LOGI("SETUP", "Comenzando SETUP.");

//     esp_log_level_set("*", ESP_LOG_INFO); // TODO: INVESTIGAR XQ esp_log_level_set NO HACE NADA EN ABSOLUTO.
// #ifdef DEBUG_ESP32
//     esp_log_level_set("*", ESP_LOG_DEBUG);
// #endif

    // DESCOMENTAR DURANTE DESARROLLO SI TODAVIA NO TE DUELE LO SUFICIENTE LA CABEZA.
    buzzer.init();
    // buzzer.beep(500);

    // Initialize the kinematic filter (Adjust mass and pad offset as needed for your launch)
    // TODO: FALTA MODIFICAR DATAFILTER DE FORMA ACORDE A LOS REQUERIMIENTOS.
    DataFilter::init();
    Sensors::init();
    GSE::init();

    // DATA DISTRIBUTOR
    xDataFilterRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xDataFilterRingbuf == NULL) {
        ESP_LOGE(TAG_TASK_DATA_FILTER, "Error al crear xDataFilterRingbuf");
    } else {
        ESP_LOGI(TAG_TASK_DATA_FILTER, "xDataFilterRingbuf creado");
    }

    // MDE
    // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
    xStateMachineRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xStateMachineRingbuf == NULL) {
        ESP_LOGE(TAG_TASK_STATE_MACHINE, "Error al crear xStateMachineRingbuf");
    } else {
        ESP_LOGI(TAG_TASK_STATE_MACHINE, "xStateMachineRingbuf creado");
    }

    // LORA
    xLoraRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (xLoraRingbuf == NULL) {
        ESP_LOGE(TAG_TASK_LORA, "Error al crear xLoraRingbuf");
    } else {
        ESP_LOGI(TAG_TASK_LORA, "xLoraRingbuf creado");
    }

    // FLASH
    // xFlashRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    // if (xFlashRingbuf == NULL) {
    //     ESP_LOGE(TAG_FLASH, "Error al crear xFlashRingbuf");
    // } else {
    //     ESP_LOGI(TAG_FLASH, "xFlashRingbuf creado");
    // }

    // TODO: Hacer un Profile. Ver si es overkill usar 4096 WORDs para esto. Ojo: WORD = 4 bits en la esp32. "You can use uxTaskGetStackHighWaterMark() to monitor unused stack space"
    xTaskCreatePinnedToCore(vTaskReadSensors, "ReadSensors", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskReadSensorsHandle), 1);
    xTaskCreatePinnedToCore(vTaskStateMachine, "StateMachine", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskStateMachineHandle), 1);
    xTaskCreatePinnedToCore(vTaskDataFilter, "DataFilter", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskDataFilterHandle), 1);

    // xTaskCreatePinnedToCore(vTaskFlash, "Flash", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskFlashHandle), 0);
    xTaskCreatePinnedToCore(vTaskLora, "Lora", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskLoraHandle), 0);

    vTaskDelete(NULL); // NULL hace referencia al task default que maneja a "void loop()"

    // buzzer.playSuccess();

    delay(100);
}

void loop(){
    // NO SE USA ESTO
}

// TODO: Pensar sobre este texto: "You need to gather large bursts of hardware data inside an Interrupt Service Routine (ISR) to be processed later by a task."
void vTaskReadSensors(void *pvParameters) {
    // const TickType_t xFrequency = pdMS_TO_TICKS(7); // TODO: ~6.7 ms → 150 Hz --> frecuencia de rafagas --> es en realidad req de vTaskLora
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_SENSORS, "Core ID: %d", xPortGetCoreID());

        // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
        data_raw_t raw = Sensors::get_raw_data();

        // print_data_raw(&raw);

        if (xDataFilterRingbuf != NULL) {
            if (xRingbufferSend(xDataFilterRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(50)) != pdTRUE) {
                ESP_LOGE(TAG_TASK_SENSORS, "xRingbufferSend -> xDataFilterRingbuf failed (raw)");
            }
        }

        // SerialPrint::plot("contadorSensores", contadorSensores);
        // contadorSensores++;

        // xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vTaskDataFilter(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_DATA_FILTER, "Core ID: %d", xPortGetCoreID());

        size_t item_size = 0;
        void *item = xRingbufferReceive(xDataFilterRingbuf, &item_size, pdMS_TO_TICKS(2000));

        if (item != NULL) {
            if (item_size == sizeof(data_raw_t)) {

                const data_raw_t *raw_ptr = static_cast<data_raw_t *>(item);
                data_all_t all_data = DataFilter::process(*raw_ptr);
                // print_data(&all_data);

                // MDE
                if (xStateMachineRingbuf != NULL) {
                    if (xRingbufferSend(xStateMachineRingbuf, (void *)&all_data, sizeof(data_all_t), pdMS_TO_TICKS(30)) != pdTRUE) {
                        ESP_LOGE(TAG_TASK_DATA_FILTER, "xRingbufferSend -> xStateMachineRingbuf failed (all_data)");
                    }
                }

                // FLASH
                // if (xFlashRingbuf != NULL) {
                //     // if (xRingbufferSend(xFlashRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                //     //     ESP_LOGE(TAG_FLASH, "xRingbufferSend -> xFlashRingbuf failed (raw)");
                //     // }
                //     if (xRingbufferSend(xFlashRingbuf, (void *)&all_data, sizeof(data_all_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                //         ESP_LOGE(TAG_FLASH, "xRingbufferSend -> xFlashRingbuf failed (all_data)");
                //     }
                // }

                // LORA
                if (xLoraRingbuf != NULL && Cohete::SYSTEM.procesos.flujos.Sensors_a_Lora_enabled) {
                    // if (xRingbufferSend(xLoraRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(10)) != pdTRUE) {
                    //     ESP_LOGE(TAG_LORA, "xRingbufferSend -> xLoraRingbuf failed (raw)");
                    // }
                    BaseType_t res = xRingbufferSend(xLoraRingbuf, (void *)&all_data, sizeof(data_all_t), pdMS_TO_TICKS(50));
                    if (res != pdTRUE) {
                        Serial.printf("xRingbufferSend (xLoraRingbuf) ha fallado (all_data). Codigo de error:%d\n", res);
                    }
                }

            } else {
                ESP_LOGE(TAG_TASK_DATA_FILTER, "Tamaño de item no coincide con data_raw_t");
            }
            vRingbufferReturnItem(xDataFilterRingbuf, item);
        } else {
            ESP_LOGI(TAG_TASK_DATA_FILTER, "No messages (timeout)");
        }
    }
}


// Mock implementation of the State Machine task: consumes sensor messages and forwards/acts on them
void vTaskStateMachine(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_STATE_MACHINE, "Core ID: %d", xPortGetCoreID());

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
                ESP_LOGE(TAG_TASK_STATE_MACHINE, "Tamaño de item no coincide con data_all_t");
            }

            // 4. Free the memory
            vRingbufferReturnItem(xStateMachineRingbuf, item);

        } else {
            ESP_LOGI(TAG_TASK_STATE_MACHINE, "No messages (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

// Mock implementation of the Flash task: consumes items from the Flash ringbuffer and "persists" them
void vTaskFlash(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_FLASH, "Core ID: %d", xPortGetCoreID());

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
            ESP_LOGI(TAG_TASK_FLASH, "No items to persist (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Mock implementation of the Lora task: consumes items and "sends" them over LoRa
void vTaskLora(void *pvParameters) {
    (void)pvParameters;
    while (true) {

        ESP_LOGD(TAG_TASK_LORA, "Core ID: %d", xPortGetCoreID());

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
            ESP_LOGI(TAG_TASK_LORA, "No messages to send (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000)); // importante ceder tiempo si hay task priorities diferentes para que no se produzca inanicion en otras tasks
    }
}