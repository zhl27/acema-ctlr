//
// Created by zhl on 6/3/26.
//

#include "mGPS.h"

#include "SerialPrint.h"


mGPS::mGPS(int uartNum, int rx, int tx, uint32_t baud)
    : serialGPS(uartNum), rxPin(rx), txPin(tx), baudRate(baud) {}

void mGPS::begin() {
    serialGPS.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void mGPS::update() {
    // Process incoming characters from GPS UART channel
    while (serialGPS.available() > 0) {
        gps.encode(serialGPS.read());
    }

    // Output GPS telemetry only when location or satellite info updates
    if (gps.satellites.isUpdated() || gps.location.isUpdated()) {
        SerialPrint::plot("s.num", static_cast<float>(gps.satellites.value()));
        SerialPrint::plot("s.lat", static_cast<float>(gps.location.lat()));
        SerialPrint::plot("s.long", static_cast<float>(gps.location.lng()));
    }
}