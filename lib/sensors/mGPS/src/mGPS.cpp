
#include "mGPS.h"

mGPS* mGPS::_instance = nullptr;

// ==========================================
// Constructor y Destructor
// ==========================================
mGPS::mGPS(int uartNum, int rx, int tx, uint32_t baud) :
    _uartNum(uartNum), _rxPin(rx), _txPin(tx), _baud(baud),
    _last_ack_status(0xFF), _taskHandle(nullptr), _dispatcher(nullptr), _configurator(nullptr)
{
    _instance = this;

    // Limpiar buffers al arranque
    memset(&_pvt_data_rx, 0, sizeof(nav_pvt_t));
    memset(&_pvt_data, 0, sizeof(nav_pvt_t));
    memset(&_ack_payload_rx, 0, sizeof(ubx_ack_payload_t));

    // Crear herramientas de sincronización de FreeRTOS
    _ackSemaphore = xSemaphoreCreateBinary();
    _dataMutex    = xSemaphoreCreateMutex();
    _uartMutex    = xSemaphoreCreateMutex();

    // 1. Ruta de datos PVT (Posición, Velocidad, Tiempo)
    _regPvt = {
        static_cast<uint8_t>(UBX_CLASS::NAV),
        static_cast<uint8_t>(UBX_ID_NAV::PVT),
        (uint8_t*)&_pvt_data_rx,
        sizeof(nav_pvt_t),
        _onPvtReceivedStatic
    };

    // 2. Ruta para ACK-ACK (Clase 0x05, ID 0x01)
    _regAck = {
        0x05, 0x01,
        (uint8_t*)&_ack_payload_rx,
        sizeof(ubx_ack_payload_t),
        _onAckReceivedStatic
    };

    // 3. Ruta para ACK-NAK (Clase 0x05, ID 0x00)
    _regNack = {
        0x05, 0x00,
        (uint8_t*)&_ack_payload_rx,
        sizeof(ubx_ack_payload_t),
        _onNackReceivedStatic
    };

    _tablaRegistros[0] = &_regPvt;
    _tablaRegistros[1] = &_regAck;
    _tablaRegistros[2] = &_regNack;

    _dispatcher   = new UbxDispatcher(_tablaRegistros, 3);
    _configurator = new UbxConfigurator(_uartTxStatic, _waitAckStatic);
}

mGPS::~mGPS() {
    if (_taskHandle) vTaskDelete(_taskHandle);
    delete _dispatcher;
    delete _configurator;
    if (_ackSemaphore) vSemaphoreDelete(_ackSemaphore);
    if (_dataMutex)    vSemaphoreDelete(_dataMutex);
    if (_uartMutex)    vSemaphoreDelete(_uartMutex);
}

// ==========================================
// Init & Autoconfiguración Segura
// ==========================================
bool mGPS::init() {
    // VERIFICACIÓN ESTÁTICA DEL PROTOCOLO UBX
    // Si el compilador introdujo padding, se detiene la compilación para evitar basura en memoria.
    static_assert(sizeof(nav_pvt_t) == 84, "FATAL: nav_pvt_t debe medir exactamente 84 bytes sin padding!");

    if (_taskHandle != nullptr) {
        Serial.println("[GPS-COHETE] El driver ya se encuentra iniciado y en ejecución.");
        return true;
    }

    Serial.println("[GPS-COHETE] Inicializando hardware UART...");

    const uart_config_t uart_config = {
        .baud_rate = static_cast<int>(_baud),
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    const uart_port_t port = (uart_port_t)_uartNum;
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port, 1024 * 2, 0, 0, NULL, 0));

    vTaskDelay(pdMS_TO_TICKS(300));

    // 1. Upgrade de Baudio a 115200 de forma sincronizada con el hardware
    Serial.println("[GPS-COHETE] Solicitando upgrade de baudio a 115200...");
    _configurator->setPortUart(115200);

    // Esperar a que el último bit salga del registro de desplazamiento de la ESP32
    uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(50));

    uart_set_baudrate(port, 115200);
    uart_flush_input(port); // Limpiar errores de framing generados por el cambio de reloj
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Configurar Modelo Dinámico Airborne < 4G (CRÍTICO PARA AVIÓNICA)
    Serial.println("[GPS-COHETE] Configurando Modelo Dinámico Airborne_4G...");
    if (!_configurator->setDynamicModel(NAV5_DYN_MODEL::Airborne_4G)) {
        Serial.println("[ERROR FATAL] El módulo u-blox rechazó (NACK/Timeout) el modelo Airborne_4G. ¡SISTEMA NO APTO PARA VUELO!");
        return false;
    }

    // 3. Configurar Navegación a 5Hz
    Serial.println("[GPS-COHETE] Configurando tasa de navegación a 5Hz...");
    if (!_configurator->setNavigationRate(5)) {
        Serial.println("[ERROR FATAL] Falló la configuración de frecuencia de navegación.");
        return false;
    }

    // 4. Suscribir SOLO el mensaje PVT (Índice 0). ¡No suscribimos los ACKs!
    Serial.println("[GPS-COHETE] Habilitando streaming de telemetría PVT...");
    _configurator->enableRegisteredMessages(_tablaRegistros, 1);

    // 5. INICIAR TAREA DE FREERTOS EN SEGUNDO PLANO
    // Asignamos 4KB de stack, prioridad 5 (media-alta para sensorica), y anclamos al Core 1.
    BaseType_t res = xTaskCreatePinnedToCore(
        _taskLoopStatic,
        "GPS_BgTask",
        4096,
        this,
        5,
        &_taskHandle,
        1
    );

    if (res != pdPASS) {
        Serial.println("[ERROR FATAL] No se pudo crear la tarea de FreeRTOS para el GPS.");
        return false;
    }

    Serial.println("[GPS-COHETE] Inicialización exitosa. Driver operando en segundo plano.");
    return true;
}

// ==========================================
// Bucle del Driver (Ejecutado por FreeRTOS)
// ==========================================
void mGPS::_taskLoopStatic(void* arg) {
    mGPS* gps = static_cast<mGPS*>(arg);
    gps->_taskLoop();
}

void mGPS::_taskLoop() {
    Serial.println("[GPS-TASK] Tarea en segundo plano en ejecución.");
    for (;;) {
        _update();
        // Ceder el procesador brevemente para evitar saturar el Core 1 (a 115200 baudios, 5Hz, esto sobra)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void mGPS::_update() const {
    uint8_t buffer[128];
    // Protegemos el acceso de lectura al periférico UART frente a concurrencia
    if (xSemaphoreTake(_uartMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        const int len = uart_read_bytes((uart_port_t)_uartNum, buffer, sizeof(buffer), 0);
        xSemaphoreGive(_uartMutex);

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                _dispatcher->handleFSM(buffer[i]); // Inyecta bytes a la máquina de estados
            }
        }
    }
}

// ==========================================
// API de Lectura Pública (Thread-Safe)
// ==========================================
nav_pvt_t mGPS::get_gps_raw_data() const {
    nav_pvt_t copy;
    // Bloqueo ultra corto: se copia la estructura completa sin riesgo de data-tearing
    xSemaphoreTake(_dataMutex, portMAX_DELAY);
    copy = _pvt_data;
    xSemaphoreGive(_dataMutex);
    return copy;
}

// ==========================================
// Funciones Puente (C++ hacia clase estática)
// ==========================================
void mGPS::_onPvtReceivedStatic(void* data) { if (_instance) _instance->_onPvtReceived(data); }
void mGPS::_onAckReceivedStatic(void* data) { if (_instance) _instance->_onAckReceived(data); }
void mGPS::_onNackReceivedStatic(void* data) { if (_instance) _instance->_onNackReceived(data); }
void mGPS::_uartTxStatic(const uint8_t* data, const size_t len) { if (_instance) _instance->_uartTx(data, len); }
bool mGPS::_waitAckStatic(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
    return _instance ? _instance->_waitAck(cls, id, timeoutMs) : false;
}

// ==========================================
// Callbacks Internos
// ==========================================
void mGPS::_onPvtReceived(void* data) {
    const nav_pvt_t* pvt = static_cast<nav_pvt_t*>(data);
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _pvt_data = *pvt;
        xSemaphoreGive(_dataMutex);
    }
}

void mGPS::_onAckReceived(void* data) {
    _last_ack_status = 0x01; // Confirmación positiva
    xSemaphoreGive(_ackSemaphore);
}

void mGPS::_onNackReceived(void* data) {
    _last_ack_status = 0x00; // Rechazo explícito del hardware
    xSemaphoreGive(_ackSemaphore);
}

void mGPS::_uartTx(const uint8_t* data, const size_t len) const {
    if (xSemaphoreTake(_uartMutex, portMAX_DELAY) == pdTRUE) {
        uart_write_bytes((uart_port_t)_uartNum, (const char*)data, len);
        xSemaphoreGive(_uartMutex);
    }
}

bool mGPS::_waitAck(uint8_t cls, uint8_t id, const uint32_t timeoutMs) {
    xSemaphoreTake(_ackSemaphore, 0); // Limpiar semáforo residual
    _last_ack_status = 0xFF;

    const uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        _update(); // Polling manual mientras el bucle de fondo aún no ha sido lanzado

        if (xSemaphoreTake(_ackSemaphore, 0) == pdTRUE) {
            // Verificar si la respuesta corresponde exactamente a la Clase e ID solicitados
            if (_ack_payload_rx.clsID == cls && _ack_payload_rx.msgID == id) {
                if (_last_ack_status == 0x01) {
                    return true; // ACK validado
                } else if (_last_ack_status == 0x00) {
                    Serial.printf("[GPS-COHETE] NACK recibido para Clase: 0x%02X, ID: 0x%02X\n", cls, id);
                    return false; // Comando rechazado por el módulo
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.printf("[GPS-COHETE] Timeout esperando ACK para Clase: 0x%02X, ID: 0x%02X\n", cls, id);
    return false;
}