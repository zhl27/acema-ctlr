//
// Created by zhl on 6/3/26.
//

#include "mGPS.h"

#include "../utils/SerialPrint.h"


mGPS::mGPS(int uartNum, int rx, int tx, uint32_t baud)
    : serialGPS(uartNum), rxPin(rx), txPin(tx), baudRate(baud) {}

void mGPS::init() {
    serialGPS.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void mGPS::update() {
    // Process incoming characters from GPS UART channel
    while (serialGPS.available() > 0) {
        gps.encode(serialGPS.read());
        // Output GPS telemetry only when location or satellite info updates
        if (this->isLocationValid()) {
            if (gps.satellites.isUpdated() || gps.location.isUpdated()) {
                SerialPrint::plot("s.num", static_cast<float>(this->getSatellites()));
                SerialPrint::plot("s.lat", static_cast<float>(this->getLatitude()));
                SerialPrint::plot("s.long", static_cast<float>(this->getLongitude()));
            }
        }
        else {
            // This will print every few seconds until a lock is found
            static unsigned long lastMessage = 0;
            if (millis() - lastMessage > 2000) {
                SerialPrint::msg("Buscando satélites...");
                lastMessage = millis();
            }
        }
    }


}