#include "mGPS.h"

#include "config.h"
#include "data.h"

mGPS::mGPS(int uartNum, int rx, int tx, uint32_t baud)
    : _uartNum(uartNum),
      _rxPin(rx),
      _txPin(tx),
      _baud(baud), _timestamp_init(0),
      _gpsQueue(nullptr),
      _serial(nullptr),
      _taskHandle(nullptr) {
}

mGPS::~mGPS() {
    if (_taskHandle != nullptr) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
    if (_gpsQueue != nullptr) {
        vQueueDelete(_gpsQueue);
        _gpsQueue = nullptr;
    }
    if (_serial != nullptr) {
        _serial->end();
        delete _serial;
        _serial = nullptr;
    }
}

bool mGPS::init() {
    if (_serial == nullptr) {
        _serial = new HardwareSerial(_uartNum);
    }

    // NEO-7M communicates at 8N1 by default
    _serial->begin(_baud, SERIAL_8N1, _rxPin, _txPin);

    // Queue of length 1 allows using xQueueOverwrite for a thread-safe "latest value" mailbox
    if (_gpsQueue == nullptr) {
        _gpsQueue = xQueueCreate(1, sizeof(data_gps_t));
        if (_gpsQueue == nullptr) {
            return false;
        }
    }

    const BaseType_t res = xTaskCreate(_gpsTask, "mGPS_Task", 3072, this, ConfigInit::TASK_PRIORITY_COMMON, &_taskHandle);

    _timestamp_init = millis();

    return (res == pdPASS);
}

// da true solamente si chequea que hay conexion uart con el modulo gps
bool mGPS::test_connection_passed() const {
    if ((millis() - _timestamp_init) <= 5000) { // si son menos de 5 segundos, todavia no terminamos el test
        return false;
    }
    if (_gps.charsProcessed() >= 10) { // si luego de 5 segundos tenemos texto, entonces se pasó el test.
        return true;
    }
    ESP_LOGE("mGPS", "No se reciben datos del GPS. Verifica el cableado RX/TX.");
    return false;
}

data_gps_t mGPS::get_gps_raw_data() const {
    data_gps_t data = {false, 0.0, 0.0, 0, 99.9};
    if (_gpsQueue != nullptr) {
        // Peek reads without removing, allowing multiple asynchronous consumers to read the last known state
        xQueuePeek(_gpsQueue, &data, 0);
    }
    return data;
}

void mGPS::_gpsTask(void* pvParameters) {
    mGPS* self = static_cast<mGPS*>(pvParameters);
    data_gps_t data = {false, 0.0, 0.0, 0, 99.9};

    while (true) {
        while (self->_serial->available() > 0) {
            self->_gps.encode(self->_serial->read());
        }

        data.is_valid = self->_gps.location.isValid();

        data.latitude = self->_gps.location.lat();
        data.longitude = self->_gps.location.lng();

        data.satellites = self->_gps.satellites.isValid() ? self->_gps.satellites.value() : 0;
        data.hdop = self->_gps.hdop.isValid() ? self->_gps.hdop.hdop() : 99.9;

        // Overwrite the single-item queue with the latest NMEA parse results
        xQueueOverwrite(self->_gpsQueue, &data);

        // 10ms yield prevents watchdog resets and easily handles NEO-7M 1Hz/5Hz baud streams
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};