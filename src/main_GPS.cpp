#include <Arduino.h>

#include "config.h"
#include "mGPS.h"

// Instantiate mGPS: UART2, RX = GPIO 16, TX = GPIO 17, 9600 baud (NEO-7M default)
mGPS gps(2, ConfigInit::GPS_RX_PIN, ConfigInit::GPS_TX_PIN, 9600);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for native USB serial (if applicable)
  }

  Serial.println("Starting mGPS Controller Test...");

  if (!gps.init()) {
    Serial.println("FATAL: mGPS initialization failed (Task or Queue creation error). Halting.");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  Serial.println("mGPS initialized successfully. Background task running.");
  Serial.println("Note: A cold GPS module indoors may take several minutes to acquire a fix.");
  Serial.println("-------------------------------------------------------------------------");
}

void loop() {
  // Non-blocking read from the FreeRTOS mailbox queue
  data_gps_t data = gps.get_gps_raw_data();

  Serial.print("[GPS] Valid: ");
  Serial.print(data.is_valid ? "YES" : "NO ");

  Serial.print(" | Sats: ");
  if (data.satellites < 10) Serial.print(" "); // Alignment padding
  Serial.print(data.satellites);

  Serial.print(" | HDOP: ");
  Serial.print(data.hdop, 2);

  if (data.is_valid) {
    // Print 6 decimal places for standard GPS coordinate precision (~0.1m resolution)
    Serial.print(" | Lat: ");
    Serial.print(data.latitude, 6);
    Serial.print(" | Lng: ");
    Serial.print(data.longitude, 6);
  } else {
    Serial.print(" | Lat/Lng: [No Fix]");
  }

  Serial.println();

  // The background FreeRTOS task drains the UART buffer continuously every 10ms.
  // You can delay loop() as long as you want without risking UART buffer overflows or lost sentences.
  delay(1000);
}