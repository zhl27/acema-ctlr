#include <Arduino.h>
#include "UbxDispatcher.h"
#include "UbxConfigurator.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// ==========================================
// Configuraciones de Hardware
// ==========================================
#define TXD1_PIN (GPIO_NUM_26)
#define RXD1_PIN (GPIO_NUM_34)
#define BUF_SIZE (1024)

// ==========================================
// Instancias Globales
// ==========================================
SemaphoreHandle_t ackSemaphore;
uint8_t expectedClass, expectedId;
nav_pvt_t mi_pvt_data; // Estructura global donde el Dispatcher dejará los datos

// ==========================================
// Callbacks
// ==========================================

void onPvtReceived(void* data) {
    // 1. Recuperación del tipo de dato original de forma segura
    nav_pvt_t* pvt = reinterpret_cast<nav_pvt_t*>(data);
    
    // 2. Impresión completa escalando los valores a unidades físicas
    Serial.println("\n=============== PAQUETE NAV-PVT ===============");
    Serial.printf("Tiempo GPS (iTOW): %lu ms\n", pvt->iTOW);
    Serial.printf("Fecha/Hora (UTC): %02d/%02d/%04d %02d:%02d:%02d\n", 
                  pvt->day, pvt->month, pvt->year, pvt->hour, pvt->min, pvt->sec);
    
    Serial.printf("Validez: Fecha=%d, Hora=%d, Totalmente Resuelto=%d\n", 
                  pvt->valid.bits.validDate, pvt->valid.bits.validTime, pvt->valid.bits.fullyResolved);
    
    Serial.printf("Fix Type: %d (0=No fix, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=Time)\n", pvt->fixType);
    Serial.printf("Flags: GNSS Fix OK=%d, Diff Soln=%d\n", 
                  pvt->flags.bits.gnssFixOK, pvt->flags.bits.diffSoln);
    Serial.printf("Satelites usados: %d\n", pvt->numSV);
    
    Serial.printf("Latitud: %.7f grados\n", pvt->lat * 1e-7f);
    Serial.printf("Longitud: %.7f grados\n", pvt->lon * 1e-7f);
    
    Serial.printf("Altura (Elipsoide): %.3f m\n", pvt->height / 1000.0f);
    Serial.printf("Altura (Nivel de Mar): %.3f m\n", pvt->hMSL / 1000.0f);
    
    Serial.printf("Precision H (hAcc): %.3f m\n", pvt->hAcc / 1000.0f);
    Serial.printf("Precision V (vAcc): %.3f m\n", pvt->vAcc / 1000.0f);
    
    Serial.printf("Velocidad 2D (gSpeed): %.3f m/s\n", pvt->gSpeed / 1000.0f);
    Serial.printf("Rumbo (Heading): %.5f grados\n", pvt->heading * 1e-5f);
    Serial.printf("PDOP: %.2f\n", pvt->pDOP * 0.01f);
    Serial.println("===============================================");
}

void onAckReceived(void* data) {
    // Al recibir un ACK-ACK, destrabamos la tarea de configuracion
    Serial.println("[DSP] -> Señal UBX-ACK recibida del Dispatcher.");
    xSemaphoreGive(ackSemaphore);
}

// ==========================================
// Funciones de Inyección (Pegamento RTOS)
// ==========================================

void uartTx(const uint8_t* data, size_t len) {
    uart_write_bytes(UART_NUM_1, (const char*)data, len);
}

bool waitAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
    expectedClass = cls;
    expectedId = id;
    Serial.printf("[SYS] Esperando ACK para (Class: 0x%02X, ID: 0x%02X)...\n", cls, id);
    
    if (xSemaphoreTake(ackSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        Serial.println("[SYS] -> EXITO: Comando Aceptado.");
        return true;
    }
    Serial.println("[SYS] -> ERROR: Timeout expirado (NAK o Perdida de paquete).");
    return false;
}

// ==========================================
// Tarea Principal de GPS
// ==========================================

// Tabla de registros que el UbxDispatcher usará para rutear
const UbxRegMsg_t regPvt = {UBX_CLASS::NAV, UBX_ID_NAV::PVT, (uint8_t*)&mi_pvt_data, sizeof(nav_pvt_t), onPvtReceived};
const UbxRegMsg_t regAck = {UBX_CLASS::ACK, 0x01, nullptr, 0, onAckReceived}; // ACK-ACK
const UbxRegMsg_t* tablaRegistros[] = {&regPvt, &regAck};

void gps_task(void* pvParameters) {
    Serial.println("\n[TASK] --- Iniciando Tarea FreeRTOS de GPS ---");

    // 1. Instanciamos las clases de lógica
    UbxDispatcher dispatcher(tablaRegistros, 2);
    UbxConfigurator configurator(uartTx, waitAck);

    // 2. Configuración nativa del driver UART en ESP-IDF
    Serial.println("[SYS] Inicializando UART_1 en ESP32 a 9600 baudios (Modo Seguro)...");
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Damos tiempo al sistema a estabilizarse
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3. Orquestación de la Configuración u-blox
    Serial.println("\n[SYS] --- Comenzando Configuración del Módulo u-blox ---");
    
    Serial.println("[CFG] 1. Solicitando cambio de baudio a 115200 y filtrado NMEA...");
    configurator.setPortUart(115200); 
    // NOTA: No importa si retorna false por timeout, ya que una vez que cambia el baudio, 
    // el GPS no logrará enviar el ACK a nuestra UART que sigue a 9600.

    // Alineamos nuestra UART local con el nuevo baudio del GPS
    Serial.println("[SYS] Reconfigurando UART_1 local a 115200 baudios...");
    vTaskDelay(pdMS_TO_TICKS(100)); // Esperamos que se vacie el tubo de 9600
    uart_set_baudrate(UART_NUM_1, 115200);
    uart_flush(UART_NUM_1); // Limpiamos buffer de posibles basuras asincronas
    vTaskDelay(pdMS_TO_TICKS(100)); 

    Serial.println("[CFG] 2. Configurando Modelo Dinámico (Airborne 4G)...");
    configurator.setDynamicModel(NAV5_DYN_MODEL::Airborne_4G);

    Serial.println("[CFG] 3. Configurando Tasa de Medición (5Hz)...");
    configurator.setNavigationRate(5);

    Serial.println("[CFG] 4. Habilitando tabla de mensajes de Telemetría...");
    configurator.enableRegisteredMessages(tablaRegistros, 2);

    Serial.println("\n[SYS] --- Sistema configurado. Entrando al bucle de Dispatcher ---");

    // 4. Bucle infinito del Task: Ingesta de bytes por Interrupción de Hardware (driver)
    uint8_t buffer[128];
    for(;;) {
        // Bloqueo eficiente en RTOS. Espera pasivamente bytes del hardware.
        int len = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                dispatcher.handleFSM(buffer[i]); // Inyección a la máquina de estados
            }
        }
    }
}

// ==========================================
// Funciones Arduino Nativas
// ==========================================

void setup() {
    // Consola de Depuración
    Serial.begin(115200);
    while(!Serial) {;} 
    
    Serial.println("\n=========================================");
    Serial.println("   Arranque: Undimotriz Telemetry Sys    ");
    Serial.println("=========================================");

    // Inicializamos el semáforo binario
    ackSemaphore = xSemaphoreCreateBinary();
    
    // Lanzamos la tarea anclada al Core 1 (El Core 0 se suele usar para WiFi/Radio)
    xTaskCreatePinnedToCore(
        gps_task,       // Función
        "GPS_Task",     // Nombre visible en el scheduler
        8192,           // Tamaño de Pila (Stack)
        NULL,           // Parámetros
        5,              // Prioridad
        NULL,           // Handle
        1               // Core
    );
}

void loop() {
    // La tarea principal en Arduino simplemente parpadeará o dormirá.
    // Toda la carga pesada está delegada en gps_task.
    vTaskDelay(pdMS_TO_TICKS(1000));
}