/**
 * @file main_GPS.cpp
 * @brief Sistema de Aviónica y Telemetría GNSS para Cohetería de Alta Potencia (HPR).
 * @details Implementa una arquitectura concurrente en FreeRTOS con aislamiento
 * estricto de hardware y protección atómica de datos (NASA Power of Ten compliant).
 */

#include <Arduino.h>
#include "UbxDispatcher.h"
#include "UbxConfigurator.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <SPI.h>

// ==========================================
// 1. Configuraciones de Hardware y Pines
// ==========================================
#define TXD1_PIN        (GPIO_NUM_17)
#define RXD1_PIN        (GPIO_NUM_16)
#define BUF_SIZE        (1024)
#define BAUD_INITIAL    (9600)
#define BAUD_TARGET     (115200)


// ==========================================
// 2. Recursos Globales de Sincronización (RTOS)
// ==========================================
SemaphoreHandle_t ackSemaphore = NULL;
QueueHandle_t     pvtQueue     = NULL; // Cola de 1 elemento para paso de datos por copia

// Buffer local donde el UbxDispatcher ensambla el paquete antes de empujarlo a la cola
nav_pvt_t mi_pvt_data_rx;
uint8_t   expectedClass, expectedId;
ubx_ack_payload_t ack_payload;

// ==========================================
// 3. Callbacks del Protocolo u-blox
// ==========================================

/**
 * @brief Callback disparado por UbxDispatcher al recibir un paquete NAV-PVT íntegro.
 * @note  Se ejecuta dentro del contexto de gps_rx_task.
 */
void onPvtReceived(void* data) {
    nav_pvt_t* pvt = reinterpret_cast<nav_pvt_t*>(data);

    // PILAR 1: Protección contra Data Tearing.
    // Sobrescribe la cola atómicamente en O(1) sin bloquear al productor ni al consumidor.
    xQueueOverwrite(pvtQueue, pvt);

#if defined(PRINT_DATA)
    ESP_LOGI("main_GPS", "\n[RX] >>> Paquete NAV-PVT actualizado en Cola de Aviónica <<<");
    ESP_LOGI("main_GPS", "Fix: %d | Satélites: %d | Altitud MSL: %.2f m | Vel 2D: %.2f m/s\n",
                  pvt->fixType, pvt->numSV, pvt->hMSL / 1000.0f, pvt->gSpeed / 1000.0f);
#endif
}

/**
 * @brief Callback disparado al recibir confirmación UBX-ACK-ACK del módulo GPS.
 */
void onAckReceived(void* data) {
    // Convertimos el buffer al tipo de dato real del ACK
    const ubx_ack_payload_t* ack = static_cast<ubx_ack_payload_t*>(data);

    // Validamos que el módulo esté confirmando exactamente la Clase e ID solicitados
    if (ack->clsID == expectedClass && ack->msgID == expectedId) {
        Serial.printf("[RX] -> Señal UBX-ACK legítima detectada (Class: 0x%02X, ID: 0x%02X). Liberando semáforo...\n", ack->clsID, ack->msgID);
        xSemaphoreGive(ackSemaphore);
    } else {
        Serial.printf("[RX] -> ACK ignorado (Recibió confirmación de 0x%02X-0x%02X pero esperaba 0x%02X-0x%02X)\n", ack->clsID, ack->msgID, expectedClass, expectedId);
    }
}

// ==========================================
// 4. Tabla de Enrutamiento del Dispatcher
// ==========================================
// ==========================================
// 4. Tabla de Enrutamiento del Dispatcher
// ==========================================
const UbxRegMsg_t regPvt = {static_cast<uint8_t>(UBX_CLASS::NAV), static_cast<uint8_t>(UBX_ID_NAV::PVT), reinterpret_cast<uint8_t *>(&mi_pvt_data_rx), sizeof(nav_pvt_t), onPvtReceived};

// CORREGIDO: Le pasamos el puntero a ack_payload y su longitud real (2 bytes)
const UbxRegMsg_t regAck = {static_cast<uint8_t>(UBX_CLASS::ACK), 0x01, reinterpret_cast<uint8_t *>(&ack_payload), sizeof(ubx_ack_payload_t), onAckReceived};

const UbxRegMsg_t* tablaRegistros[] = {&regPvt, &regAck};

// Instancia global del Dispatcher (único consumidor del stream de bytes RX)
UbxDispatcher dispatcher(tablaRegistros, 2);

// ==========================================
// 5. Funciones de Inyección (Hardware Abstraction)
// ==========================================

void uartTx(const uint8_t* data, size_t len) {
    // PILAR 2: El driver nativo de ESP-IDF es thread-safe y opera en Full-Duplex real
    uart_write_bytes(UART_NUM_1, (const char*)data, len);
}

bool waitAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
    expectedClass = cls;
    expectedId = id;
    ESP_LOGI("main_GPS", "[CFG] Esperando ACK para (Class: 0x%02X, ID: 0x%02X)...\n", cls, id);

    // PILAR 4: Purgado de semáforo sucio.
    // Eliminamos cualquier ACK "fantasma" que haya llegado fuera de tiempo previamente.
    xSemaphoreTake(ackSemaphore, 0);

    // Nos bloqueamos pasivamente hasta que la tarea RX libere el semáforo o venza el timeout
    if (xSemaphoreTake(ackSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        ESP_LOGI("main_GPS", "[CFG] -> ÉXITO: Comando Aceptado por el módulo.");
        return true;
    }
    ESP_LOGE("main_GPS", "[CFG] -> ERROR: Timeout expirado (NAK o pérdida de paquete).");
    return false;
}

// ==========================================
// 6. Tarea FreeRTOS: Ingesta Continua (RX)
// ==========================================
/**
 * @brief Tarea de alta prioridad dedicada exclusivamente a leer el hardware UART.
 * @note  Al estar separada, garantiza que nunca se pierdan bytes ni ACKs mientras se configura.
 */
void gps_rx_task(void* pvParameters) {
    ESP_LOGI("main_GPS", "[TASK_RX] --- Iniciando demonio de lectura UART (5Hz / 115200) ---");
    uint8_t buffer[128];

    for(;;) {
        // Lectura bloqueante eficiente. Despierta de inmediato si hay bytes o cada 10ms
        int len = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer), pdMS_TO_TICKS(10));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                // Inyectamos byte a byte en la máquina de estados
                dispatcher.handleFSM(buffer[i]);
            }
        }
    }
}

// ==========================================
// 7. Tarea FreeRTOS: Secuencia de Configuración
// ==========================================
/**
 * @brief Tarea secuencial que reconfigura el GPS para vuelo (Modelo Airborne 4G).
 * @note  Una vez completada su misión, se autodestruye para liberar memoria Stack.
 */
void gps_config_task(void* pvParameters) {
    ESP_LOGI("main_GPS", "\n[TASK_CFG] --- Comenzando Secuencia de Configuración u-blox ---");
    const UbxConfigurator configurator(uartTx, waitAck);

    // 1. Solicitud de cambio de baudio a 115200 y filtrado NMEA
    ESP_LOGI("main_GPS", "[CFG] 1. Solicitando cambio de baudio de %d a %d...\n", BAUD_INITIAL, BAUD_TARGET);
    configurator.setPortUart(BAUD_TARGET);

    // PILAR 3: Transición Blindada de Baudio por Hardware.
    // a) Esperamos físicamente a que el último bit del comando salga por el pin TX
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100));

    // b) Cambiamos la frecuencia de reloj del periférico local en el ESP32
    ESP_LOGI("main_GPS", "[SYS] Reconfigurando reloj de UART_1 local a 115200 baudios...");
    uart_set_baudrate(UART_NUM_1, BAUD_TARGET);

    // c) Limpiamos basuras electromagnéticas generadas durante la asincronía del cambio
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush_input(UART_NUM_1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Configuración de Modelo Dinámico para Cohetería (<4G Airborne)
    ESP_LOGI("main_GPS", "[CFG] 2. Configurando Modelo Dinámico (Airborne 4G)...");
    configurator.setDynamicModel(NAV5_DYN_MODEL::Airborne_4G);

    // 3. Configuración de Tasa de Navegación (5Hz = 200ms)
    ESP_LOGI("main_GPS", "[CFG] 3. Configurando Tasa de Medición (5 Hz)...");
    configurator.setNavigationRate(5);

    // 4. Activación de Mensajes NAV-PVT en el módulo
    ESP_LOGI("main_GPS", "[CFG] 4. Habilitando stream de telemetría NAV-PVT...");
    configurator.enableRegisteredMessages(tablaRegistros, 2);

    ESP_LOGI("main_GPS", "\n[SYS] === AVIONICA GNSS CONFIGURADA Y EN VUELO ===");

    // Autodestrucción de la tarea de configuración para liberar RAM al sistema
    vTaskDelete(NULL);
}

// ==========================================
// 8. Setup y Arranque del Sistema
// ==========================================

void setup() {
    Serial.begin(115200);
    while(!Serial) {;}

    ESP_LOGI("main_GPS", "===  FLIGHT COMPUTER: TELEMETRY & GNSS SYS  ===");

    ackSemaphore = xSemaphoreCreateBinary();
    pvtQueue     = xQueueCreate(1, sizeof(nav_pvt_t)); // Cola de 1 posición para sobrescritura

    configASSERT(ackSemaphore != NULL);
    configASSERT(pvtQueue != NULL);

    // 2. Configuración nativa del driver UART en ESP-IDF a 9600 baudios iniciales
    ESP_LOGI("main_GPS", "[SYS] Instalando driver UART_1 en Modo Seguro (9600 8N1)...");
    uart_config_t uart_config = {
        .baud_rate  = BAUD_INITIAL,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));

    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. Lanzamiento de Tareas Concurrentes en Core 1
    // Prioridad 6 para RX (Máxima prioridad de I/O para no perder bytes de telemetría)
    xTaskCreatePinnedToCore(gps_rx_task,     "GPS_RX_Task",  4096, NULL, 6, NULL, 1);

    // Prioridad 5 para Configuración (Espera al semáforo liberado por RX)
    xTaskCreatePinnedToCore(gps_config_task, "GPS_Cfg_Task", 4096, NULL, 5, NULL, 1);
}

// ==========================================
// 9. Bucle Principal (Simulación de Computadora de Vuelo / LoRa)
// ==========================================

void loop() {
    nav_pvt_t datosVuelo;

    // Consumo de datos sin bloqueo (Thread-Safe).
    // Aquí tu máquina de estados de altitud, paracaídas y LoRa leerán la información:
    if (xQueueReceive(pvtQueue, &datosVuelo, 0) == pdTRUE) {

        // Ejemplo: Si el cohete supera los 1000m y empieza a descender -> Disparar recuperación
        /*
        float altitudActual = datosVuelo.hMSL / 1000.0f;
        if (altitudActual < altitudMaximaAlcanzada && vueloEnCurso) {
            dispararParacaidas();
        }
        */
    }

    // El loop corre libremente para otras tareas (sensores BMP280, MPU6050, LoRa, etc.)
    vTaskDelay(pdMS_TO_TICKS(10));
}