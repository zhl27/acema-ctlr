//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_MGPS_H
#define ACEMA_CTLR_MGPS_H
#include <cstdint>

#include "TinyGPS++.h"


/**
 * @class mGPS
 * @brief Digital Twin for the TinyGPSPlus GPS receiver module.
 *
 * Encapsulates the serial communication, parser encoding, and outputs coordinate
 * and satellite counts to telemetry whenever changes are detected.
 */
class mGPS {
private:
    TinyGPSPlus gps;
    HardwareSerial serialGPS;
    int rxPin;
    int txPin;
    uint32_t baudRate;

public:
    /**
     * @brief Constructs mGPS with configuration for ESP32 HardwareSerial.
     * @param uartNum Hardware UART index (default 2)
     * @param rx Rx pin (default 16)
     * @param tx Tx pin (default 17)
     * @param baud Baud rate (default 9600)
     */
    mGPS(int uartNum = 2, int rx = 16, int tx = 17, uint32_t baud = 9600);

    /**
     * @brief Initializes Serial connection.
     */
    void begin();

    /**
     * @brief Reads incoming serial data, feeds the GPS decoder, and publishes updates when new data is available.
     */
    void update();

    // Getters
    uint32_t getSatellites() { return gps.satellites.value(); }
    double getLatitude() { return gps.location.lat(); }
    double getLongitude() { return gps.location.lng(); }
    bool isLocationValid() const { return gps.location.isValid(); }

    // Direct access to the parser object
    TinyGPSPlus& getRawGPS() { return gps; }
};



#endif //ACEMA_CTLR_MGPS_H
