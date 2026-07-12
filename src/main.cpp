#include <SPI.h>
#include "LoraWrapped.h"

#include "main.h"
#include <freertos/queue.h>

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

    // --- DATA DISTRIBUTOR ---
    // Crea una cola capaz de alojar hasta BUF_Q_SENSOR_SIZE muestras de tipo data_raw_t.
    xColaSensores = xQueueCreate(BUF_Q_SENSOR_SIZE, sizeof(data_raw_t));

    if (xColaSensores == NULL) {
        ESP_LOGE(TAG_TASK_DATA_FILTER, "Error al crear xColaSensores");
    } else {
        ESP_LOGI(TAG_TASK_DATA_FILTER, "xColaSensores creada correctamente");
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
    xTaskCreate(vTaskReadSensors, "ReadSensors", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskReadSensorsHandle));
    xTaskCreate(vTaskStateMachine, "StateMachine", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskStateMachineHandle));
    // xTaskCreate(vTaskFlash, "Flash", 4096, NULL, 3, &xTaskFlashHandle);
    xTaskCreate(vTaskLora, "Lora", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskLoraHandle));
    xTaskCreate(vTaskDataFilter, "DataFilter", 4096, NULL, 4, &(Cohete::SYSTEM.procesos.xTaskDataFilterHandle));

    vTaskDelete(NULL); // NULL hace referencia al task default que maneja a "void loop()"

    // buzzer.playSuccess();

    delay(100);
}

void loop(){
    // NO SE USA ESTO
}

// TODO: Pensar sobre este texto: "You need to gather large bursts of hardware data inside an Interrupt Service Routine (ISR) to be processed later by a task."
void vTaskReadSensors(void *pvParameters) {
    

    // ---------------------------------------------------------------------------------
    // Timer de muestreo
    // ---------------------------------------------------------------------------------
    TickType_t xLastWakeTime;
    const TickType_t xPeriodo = pdMS_TO_TICKS(PERIOD_SAMPLIG_SENSORS_MS); // Muestreo cada 10 ms (100 Hz)
  
    // Inicializar el tiempo de referencia para vTaskDelayUntil
    xLastWakeTime = xTaskGetTickCount();

    (void)pvParameters;
    while (true) {
        // Espera estricta y precisa hasta el próximo ciclo de 10ms
        vTaskDelayUntil(&xLastWakeTime, xPeriodo);

        ESP_LOGD(TAG_TASK_SENSORS, "Core ID: %d", xPortGetCoreID());

        // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
        data_raw_t raw = Sensors::get_raw_data();

        // Timestamp con esp nativo
        raw.timestamp_us = esp_timer_get_time();
        // print_data_raw(&raw);

        // 3. Enviar a la cola del Filtro de Kalman de forma NO bloqueante (Timeout = 0)
        // Si la cola se llena porque la se retrasó, preferimos perder una muestra 
        // antes que congelar el temporizador de 10ms de los sensores.
        if (xColaSensores != NULL) {
            if (xQueueSend(xColaSensores, &raw, 0) != pdTRUE) {
                // Si falla, registramos un Warning de telemetría (la cola está saturada)
                ESP_LOGW(TAG_TASK_SENSORS, "Cola llena: Muestra descartada para proteger el timing.");
                // SI VEMOS ESTE ERROR, HAY QUE AUMENTAR EL TAMÑO DE LA COLA
            }
        }

        // SerialPrint::plot("contadorSensores", contadorSensores);
        // contadorSensores++;

        // Simulate a 1 second sampling interval
        // vTaskDelay(pdMS_TO_TICKS(1000)); // it yields CPU to lower priorities for 1s
    }
}

void vTaskDataFilter(void *pvParameters) {
    data_raw_t datoRecibido;
    
    int64_t timestamp_anterior = 0;
    bool es_primera_muestra = true;

    (void)pvParameters;

    while (true) {
        // T2 se duerme hasta que llegue el paquete crudo desde la Queue de T1
        if (xQueueReceive(xColaSensores, &datoRecibido, portMAX_DELAY) == pdTRUE) {
            
            if (es_primera_muestra) {
                timestamp_anterior = datoRecibido.timestamp_us;
                es_primera_muestra = false;
                continue;
            }

            // 1. Cálculo del Delta T (dt)
            int64_t diferencia_tiempo = datoRecibido.timestamp_us - timestamp_anterior;
            float delta_t = (float)diferencia_tiempo / 1000000.0f;
            timestamp_anterior = datoRecibido.timestamp_us;

            // -------------------------------------------------------------------------
            // 2. MAPEO DE EJES (Sensor MPU -> Cohete Físico)
            // Según mMPU6050.cpp, el eje X del sensor apunta hacia la nariz del cohete
            // -------------------------------------------------------------------------
            // Aceleraciones en Gs respecto a la estructura del cohete
            float cohete_accel_z = datoRecibido.mpu.accel_x / 9.80665f; // Vertical real del cohete (Eje X del MPU)
            float cohete_accel_y = datoRecibido.mpu.accel_y / 9.80665f; // Lateral (Eje Y del MPU)
            float cohete_accel_x = datoRecibido.mpu.accel_z / 9.80665f; // Lateral (Eje Z del MPU)

            // Velocidades angulares en °/s respecto a la estructura del cohete
            // (Si la nariz es X en el MPU, entonces rotar sobre X es el Roll del cohete)
            float cohete_gyro_yaw   = datoRecibido.mpu.gyro_x * 57.2958f; // Rotación sobre el eje vertical
            float cohete_gyro_pitch = datoRecibido.mpu.gyro_y * 57.2958f; // Cabeceo
            float cohete_gyro_roll  = datoRecibido.mpu.gyro_z * 57.2958f; // Alabeo

            // -------------------------------------------------------------------------
            // 3. TRIGONOMETRÍA Y FILTRADO (Usando los ejes mapeados del cohete)
            // -------------------------------------------------------------------------
            float accel_pitch = atan2(cohete_accel_x, cohete_accel_z) * 57.2958f;
            float accel_yaw   = atan2(cohete_accel_y, cohete_accel_z) * 57.2958f;

            // Filtro Kalman Dinámico (usamos cohete_accel_z que tiene 1G en rampa)
            float pitch_filtrado = kalmanPitch.update(cohete_gyro_pitch, accel_pitch, delta_t, cohete_accel_z);
            float yaw_filtrado   = kalmanYaw.update(cohete_gyro_yaw, accel_yaw, delta_t, cohete_accel_z);

            // 4. Cálculo del Ángulo Total (Inclinación respecto a la vertical del cielo)
            float pitch_rad = pitch_filtrado * (M_PI / 180.0f);
            float yaw_rad   = yaw_filtrado * (M_PI / 180.0f);
            float inclinacion_rad = acos(cos(pitch_rad) * cos(yaw_rad));
            float inclinacion_total_grados = inclinacion_rad * (180.0f / M_PI);

            // -------------------------------------------------------------------------
            // 5. EMPAQUETADO FINAL (Estructura de data_all_t)
            // -------------------------------------------------------------------------
            data_all_t all_data = {}; // Inicializamos en 0
            
            // Velocidades angulares directas (°/s)
            all_data.vel_angular_x = cohete_gyro_pitch; 
            all_data.vel_angular_y = cohete_gyro_roll;
            all_data.vel_angular_z = cohete_gyro_yaw;

            // Ángulos absolutos filtrados (°)
            all_data.angulo_pitch = pitch_filtrado;
            all_data.angulo_yaw = yaw_filtrado;
            all_data.angulo_respecto_z = inclinacion_total_grados;

            // ... (Aquí mapearás el resto de variables: Barómetro, GPS, etc.) ...
                    
            // 6. DISTRIBUCIÓN A TAREAS (MdE, Lora)
            if (xStateMachineRingbuf != NULL) {
                xRingbufferSend(xStateMachineRingbuf, (void *)&all_data, sizeof(data_all_t), 0);
            }
            if (xLoraRingbuf != NULL) {
                xRingbufferSend(xLoraRingbuf, (void *)&all_data, sizeof(data_all_t), 0);
            }
        }
        
    }
}

/*
void vTaskDataFilter(void *pvParameters) {
    (void)pvParameters;
    
    while (true) {
        ESP_LOGD(TAG_TASK_DATA_FILTER, "Core ID: %d", xPortGetCoreID());
        // Dentro de tareaKalman
        data_raw_t datoRecibido;

        // Se duerme aquí hasta que llegue un dato, liberando CPU para otras tareas.
        if (xQueueReceive(xColaSensores, &datoRecibido, portMAX_DELAY) == pdTRUE) {
            // Aquí procesas tu datoRecibido con el Filtro de Kalman
            // ...
        }
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
*/

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