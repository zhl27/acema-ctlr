#include <SPI.h>
#include "LoraWrapped.h"

#include "main.cpp.h"

#include "services/Actuators.h"

void setup() {
    Serial.begin(115200); // TODO: Para la Compu de vuelo no se usa Serial

    while (!Serial)
        delay(1000);

    ESP_LOGI("SETUP", "Comenzando SETUP.");

//     esp_log_level_set("*", ESP_LOG_INFO); // TODO: INVESTIGAR XQ esp_log_level_set NO HACE NADA EN ABSOLUTO.
//     esp_log_level_set("*", ESP_LOG_DEBUG);

    // DataFilter::init(); // TODO: Encapsular lógica de filtros de kalman dentro de DataFilter. Ahora mismo no se usa esta clase. Pero debería utilizarse para ocultar complejidad de filtros de kalman y afines.
    Sensors::init();
    Actuators::init();
    GSE::init();

    // DESCOMENTAR DURANTE DESARROLLO SI TODAVIA NO TE DUELE LO SUFICIENTE LA CABEZA.
    // Actuators::getBuzzer().beep(500);


    // --- DATA DISTRIBUTOR ---
    // Crea una cola capaz de alojar hasta BUF_Q_SENSOR_SIZE muestras de tipo data_raw_t.
    xColaSensores = xQueueCreate(BUF_Q_SENSOR_SIZE, sizeof(data_raw_t));

    if (xColaSensores == NULL) {
        ESP_LOGE(TAG_TASK_DATA_FILTER, "Error al crear xColaSensores");
    } else {
        ESP_LOGI(TAG_TASK_DATA_FILTER, "xColaSensores creada correctamente");
    }
    // xDataFilterRingbuf = xRingbufferCreate(RBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    // if (xDataFilterRingbuf == NULL) {
    //     ESP_LOGE(TAG_TASK_DATA_FILTER, "Error al crear xDataFilterRingbuf");
    // } else {
    //     ESP_LOGI(TAG_TASK_DATA_FILTER, "xDataFilterRingbuf creado");
    // }

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

    // ---------------------------------------------------------------------------------
    // Timer de muestreo
    // ---------------------------------------------------------------------------------
    TickType_t xLastWakeTime;
    const TickType_t xPeriodo = pdMS_TO_TICKS(PERIOD_SAMPLIG_SENSORS_MS); // Muestreo cada 10 ms (100 Hz)

    // Inicializar el tiempo de referencia para vTaskDelayUntil
    xLastWakeTime = xTaskGetTickCount();

    (void)pvParameters;
    while (true) {
        // Espera estricta y precisa hasta el próximo ciclo de 10ms --> ademas nos permite procesar a las otras Tasks
        vTaskDelayUntil(&xLastWakeTime, xPeriodo);

        ESP_LOGD(TAG_TASK_SENSORS, "Core ID: %d", xPortGetCoreID());

        // TODO: Para los tasks que consumen más lento, deberíamos poner buffers más grandes. RBUF_SIZE quizás haya que borrarlo.
        data_raw_t raw = Sensors::get_raw_data();
        // print_data_raw(&raw);

        // TODO: Curiosidad: Por qué se utiliza una Queue en lugar de un Ringbuffer ?

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
        // if (xDataFilterRingbuf != NULL) {
        //     if (xRingbufferSend(xDataFilterRingbuf, (void *)&raw, sizeof(data_raw_t), pdMS_TO_TICKS(50)) != pdTRUE) {
        //         ESP_LOGE(TAG_TASK_SENSORS, "xRingbufferSend -> xDataFilterRingbuf failed (raw)");
        //     }
        // }

        // SerialPrint::plot("contadorSensores", contadorSensores);
        // contadorSensores++;

        // xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



// acá se realiza la depuración de los datos.
void vTaskDataFilter(void *pvParameters)
{
    static Kalman2D kalmanAlt;
    static bool kalmanAltInit = false;

    static EmaFilter emaTemperatura(FREC_SAMPLING_SENSORS_HZ, 1.0f);
    static EmaFilter emaPresion(FREC_SAMPLING_SENSORS_HZ, 2.0f);
    static EmaFilter emaDensidad(FREC_SAMPLING_SENSORS_HZ, 1.0f);
    static EmaFilter emaAccelVertical(FREC_SAMPLING_SENSORS_HZ, 15.0f);
    data_raw_t raw;

    int64_t timestampAnterior = 0;
    bool primeraMuestra = true;

    (void) pvParameters;

    while (true)
    {
        if (xQueueReceive(xColaSensores, &raw, portMAX_DELAY) == pdTRUE){

            print_data_raw(&raw);

            //----------------------------------------------------------------------
            // Primera muestra: solamente inicializa el tiempo
            //----------------------------------------------------------------------
            if (primeraMuestra)
            {
                timestampAnterior = raw.timestamp_us;
                primeraMuestra = false;
                continue;
            }

            //----------------------------------------------------------------------
            // detal t
            //----------------------------------------------------------------------
            const float dt = (raw.timestamp_us - timestampAnterior) * 1e-6f;

            timestampAnterior = raw.timestamp_us;

            //----------------------------------------------------------------------
            // MAPEO DE EJES (Sensor MPU -> Cohete Físico)
            // Todo permanece en las unidades nativas
            //----------------------------------------------------------------------
            const float accelX_g = raw.mpu.accel_z_g;
            const float accelY_g = raw.mpu.accel_y_g;
            const float accelZ_g = raw.mpu.accel_x_g;

            const float gyroRoll_rad_s  = raw.mpu.gyro_z_rad_s;     // Alabeo
            const float gyroPitch_rad_s = raw.mpu.gyro_y_rad_s;     // Cabeceo
            const float gyroYaw_rad_s   = raw.mpu.gyro_x_rad_s;     // Rotación sobre el eje vertical

            //----------------------------------------------------------------------
            // Ángulos obtenidos del acelerómetro.
            //----------------------------------------------------------------------
            const float accelPitch_rad = atan2f(accelX_g, accelZ_g);
            const float accelYaw_rad   = atan2f(accelY_g, accelZ_g);

            //----------------------------------------------------------------------
            // Kalman 1D
            //
            // Todo el filtro trabaja en: rad, rad/s, G
            //----------------------------------------------------------------------
            const float pitch_rad = kalmanPitch.update( gyroPitch_rad_s, accelPitch_rad, dt, accelZ_g);

            const float yaw_rad = kalmanYaw.update( gyroYaw_rad_s, accelYaw_rad, dt, accelZ_g);

            //----------------------------------------------------------------------
            // Inclinación total respecto de la vertical. (Inclinación respecto a la vertical del cielo)
            //----------------------------------------------------------------------
            const float inclinacion_rad = acosf(cosf(pitch_rad) * cosf(yaw_rad));

            //----------------------------------------------------------------------
            // Proyección de la aceleración longitudinal sobre el eje vertical global.
            //
            // El sensor lee: A_leida = A_real + gravedad_en_eje_z
            // Entonces: A_real = A_leida - gravedad_en_eje_z
            // Donde la gravedad proyectada en el eje longitudinal es 1g * cos(inclinacion)
            // accelVertical_g continúa estando en G.
            //----------------------------------------------------------------------
            float accelVertical_g = accelZ_g * cosf(pitch_rad) * cosf(yaw_rad) - cosf(inclinacion_rad);
            accelVertical_g = emaAccelVertical.actualizar(accelVertical_g);

            //----------------------------------------------------------------------
            // Kalman 2D
            //
            // Este filtro trabaja naturalmente en SI.
            //----------------------------------------------------------------------
            constexpr float G_TO_MS2 = 9.80665f;

            const float accelVertical_m_s2 = accelVertical_g * G_TO_MS2;

            if (!kalmanAltInit) {
                kalmanAlt.init(0.0f, 0.0f, 0.5f, 0.1f);  // Ajusta sigma según tus pruebas (ruido acel, ruido baro)
                kalmanAltInit = true;
            }

            kalmanAlt.update(dt, accelVertical_m_s2, raw.bmp.altitud_snm_m);

            //----------------------------------------------------------------------
            // EMPAQUETADO
            //
            // Recién acá convertimos a las unidades públicas de data_all_t.
            //----------------------------------------------------------------------
            //constexpr float RAD_TO_DEG = 57.2957795131f;

            data_all_t out = {};

            // Velocidades angulares
            out.vel_angular_x_deg_s = gyroPitch_rad_s * RAD_TO_DEG;
            out.vel_angular_y_deg_s = gyroRoll_rad_s  * RAD_TO_DEG;
            out.vel_angular_z_deg_s = gyroYaw_rad_s   * RAD_TO_DEG;

            // Actitud
            out.angulo_pitch_deg      = pitch_rad * RAD_TO_DEG;
            out.angulo_yaw_deg        = yaw_rad * RAD_TO_DEG;
            out.angulo_respecto_z_deg = inclinacion_rad * RAD_TO_DEG;

            // Cinemática vertical
            out.altitud_filtrada_m  = kalmanAlt.getAltitude();
            out.vel_z_filtrada_m_s   = kalmanAlt.getVelocity(); // TODO: Tomar a Y como eje vertical. Por ahora, para testeos Z es eje vertical. DEBEMOS CAMBIARLO.
            out.aceleracion_z_m_s2  = accelVertical_m_s2;

            // Ambientales
            out.temperatura_amb_c   = raw.bmp.temp_deg_c;
            out.densidad_aire_kg_m3 = emaDensidad.actualizar(calcularDensidadAire(raw.bmp.presion_hpa, raw.bmp.temp_deg_c));

            // GPS
            out.gps_is_valid = raw.gps.is_valid;
            out.gps_hdop = raw.gps.hdop;
            out.gps_nro_satelites = raw.gps.satellites;
            out.gps_latitud = raw.gps.latitude;
            out.gps_longitud = raw.gps.longitude;

            print_data(&out);

            //----------------------------------------------------------------------
            // Distribución (MdE, Lora)
            //----------------------------------------------------------------------
            if (xStateMachineRingbuf != NULL)
            {
                xRingbufferSend(
                    xStateMachineRingbuf,
                    &out,
                    sizeof(data_all_t),
                    0);
            }

            if (xLoraRingbuf != NULL)
            {
                xRingbufferSend(
                    xLoraRingbuf,
                    &out,
                    sizeof(data_all_t),
                    0);
            }
        }
        else {
            ESP_LOGI(TAG_TASK_DATA_FILTER, "No messages (timeout)");
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
        // size_t item_size = 0;
        // void *item = xRingbufferReceive(xDataFilterRingbuf, &item_size, pdMS_TO_TICKS(2000));

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