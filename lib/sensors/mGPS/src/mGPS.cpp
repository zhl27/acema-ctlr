//
// Created by zhl on 6/3/26.
//

#include "mGPS.h"

// Inicialización del puntero estático para los callbacks C
mGPS* mGPS::_instance = nullptr;

// ==========================================
// Constructor y Destructor
// ==========================================
mGPS::mGPS(int uartNum, int rx, int tx, uint32_t baud) :
    _uartNum(uartNum), _rxPin(rx), _txPin(tx), _baud(baud),
    _dispatcher(nullptr), _configurator(nullptr)
{
    _instance = this; // Guardar referencia para callbacks estáticos

    // Limpiar buffers
    memset(&_pvt_data_rx, 0, sizeof(nav_pvt_t));
    memset(&_pvt_data, 0, sizeof(nav_pvt_t));

    // Crear herramientas de sincronización de FreeRTOS
    _ackSemaphore = xSemaphoreCreateBinary();
    _dataMutex = xSemaphoreCreateMutex();

    // Armar las rutas de los mensajes UBX
    _regPvt.msgClass = static_cast<uint8_t>(UBX_CLASS::NAV);
    _regPvt.msgID    = static_cast<uint8_t>(UBX_ID_NAV::PVT);
    _regPvt.payloadBuffer = (uint8_t*)&_pvt_data_rx;
    _regPvt.length   = sizeof(nav_pvt_t);
    _regPvt.onReceive = _onPvtReceivedStatic;

    _regAck.msgClass = static_cast<uint8_t>(UBX_CLASS::ACK);
    _regAck.msgID    = 0x01; // ACK-ACK
    _regAck.payloadBuffer = nullptr;
    _regAck.length   = 0;
    _regAck.onReceive = _onAckReceivedStatic;

    _tablaRegistros[0] = &_regPvt;
    _tablaRegistros[1] = &_regAck;

    // Inyectamos todo en el sistema
    _dispatcher = new UbxDispatcher(_tablaRegistros, 2);
    _configurator = new UbxConfigurator(_uartTxStatic, _waitAckStatic);
}

mGPS::~mGPS() {
    if (_dispatcher) delete _dispatcher;
    if (_configurator) delete _configurator;
    vSemaphoreDelete(_ackSemaphore);
    vSemaphoreDelete(_dataMutex);
}

// ==========================================
// Init & Update
// ==========================================
void mGPS::init() {
    Serial.println("[GPS] Inicializando UART...");

    // 1. Configuración de UART en ESP-IDF
    uart_config_t uart_config = {
        .baud_rate = (int)_baud, // Arrancamos a 9600 por defecto
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_port_t port = (uart_port_t)_uartNum;
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port, 1024 * 2, 0, 0, NULL, 0));

    delay(500);

    // 2. Configurar Módulo GPS u-blox
    Serial.println("[GPS] Solicitando upgrade de baudio a 115200...");
    _configurator->setPortUart(115200);
    delay(100);

    // Alinear nuestro propio microcontrolador al nuevo baudio
    uart_set_baudrate(port, 115200);
    uart_flush(port);
    delay(100);

    Serial.println("[GPS] Configurando Modelo Airborne_4G...");
    _configurator->setDynamicModel(NAV5_DYN_MODEL::Airborne_4G);

    Serial.println("[GPS] Configurando Navegacion a 5Hz...");
    _configurator->setNavigationRate(5);

    Serial.println("[GPS] Suscribiendo mensajes PVT...");
    _configurator->enableRegisteredMessages(_tablaRegistros, 2);

    Serial.println("[GPS] Inicializacion completa.");
}

void mGPS::update() const {
    uint8_t buffer[128];
    // Lee sin bloquear o con un bloqueo cortísimo
    const int len = uart_read_bytes((uart_port_t)_uartNum, buffer, sizeof(buffer), pdMS_TO_TICKS(1));

    if (len > 0) {
        for (int i = 0; i < len; i++) {
            _dispatcher->handleFSM(buffer[i]); // Pasa bytes a la Máquina de Estados
        }
    }
}

// ==========================================
// Getters de la clase (Thread-Safe)
// ==========================================
uint32_t mGPS::getSatellites() const {
    return get_gps_raw_data().numSV;
}

double mGPS::getLatitude() const {
    return get_gps_raw_data().lat * 1e-7;
}

double mGPS::getLongitude() const {
    return get_gps_raw_data().lon * 1e-7;
}

bool mGPS::is3dFixed() const {
    nav_pvt_t data = get_gps_raw_data();
    return (data.fixType == 3 || data.fixType == 4); // 3=3D Fix, 4=GNSS+Dead Reckoning
}

nav_pvt_t mGPS::get_gps_raw_data() const {
    nav_pvt_t copy;
    xSemaphoreTake((SemaphoreHandle_t)_dataMutex, portMAX_DELAY);
    copy = _pvt_data; // Copia segura
    xSemaphoreGive((SemaphoreHandle_t)_dataMutex);
    return copy;
}

// ==========================================
// Funciones Puente (C++ hacia clase estática)
// ==========================================
void mGPS::_onPvtReceivedStatic(void* data) {
    if (_instance) _instance->_onPvtReceived(data);
}

void mGPS::_onAckReceivedStatic(void* data) {
    if (_instance) _instance->_onAckReceived(data);
}

void mGPS::_uartTxStatic(const uint8_t* data, size_t len) {
    if (_instance) _instance->_uartTx(data, len);
}

bool mGPS::_waitAckStatic(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
    return _instance ? _instance->_waitAck(cls, id, timeoutMs) : false;
}

// ==========================================
// Lógica Interna de los Callbacks
// ==========================================
void mGPS::_onPvtReceived(void* data) {
    const nav_pvt_t* pvt = static_cast<nav_pvt_t*>(data);
    // Movemos los datos leídos del UART al buffer seguro expuesto al usuario
    xSemaphoreTake(_dataMutex, portMAX_DELAY);
    _pvt_data = *pvt;
    xSemaphoreGive(_dataMutex);
}

void mGPS::_onAckReceived(void* data) {
    xSemaphoreGive(_ackSemaphore); // Libera la pausa
}

void mGPS::_uartTx(const uint8_t* data, const size_t len) {
    uart_write_bytes((uart_port_t)_uartNum, (const char*)data, len);
}

bool mGPS::_waitAck(uint8_t cls, uint8_t id, const uint32_t timeoutMs) {
    xSemaphoreTake(_ackSemaphore, 0); // Vaciamos por seguridad
    const uint32_t start = millis();

    while ((millis() - start) < timeoutMs) {
        update(); // CRÍTICO: Procesamos los bytes del UART mientras esperamos
        if (xSemaphoreTake(_ackSemaphore, 0) == pdTRUE) {
            return true; // Éxito! Recibimos el ACK de esta clase y ID.
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("[GPS] WARNING: Timeout esperando ACK del módulo.");
    return false;
}