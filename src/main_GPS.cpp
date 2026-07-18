#include <Arduino.h>
#include <TinyGPS++.h>

#define GPS_RX_PIN 16 // Conectar al pin TX del módulo NEO-7M
#define GPS_TX_PIN 17 // Conectar al pin RX del módulo NEO-7M

// Instanciar el objeto TinyGPS++ y el puerto serie
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println("--- Test Avanzado: Parseo de datos GPS NEO-7M ---");
  Serial.println("Esperando fijación de satélites (FIX)...");
}

void loop() {
  // Alimenta el objeto GPS continuamente con los datos del puerto serie
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // Imprimir los datos cada 2 segundos
  static unsigned long ultimaImpresion = 0;
  if (millis() - ultimaImpresion > 2000) {
    ultimaImpresion = millis();

    Serial.print("Satélites conectados: ");
    Serial.println(gps.satellites.value());

    if (gps.location.isValid()) {
      Serial.print("Latitud: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" | Longitud: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Ubicación: No válida (Buscando satélites...)");
    }

    if (gps.altitude.isValid()) {
      Serial.print("Altitud: ");
      Serial.print(gps.altitude.meters());
      Serial.println(" metros");
    }

    if (gps.time.isValid()) {
      Serial.print("Hora UTC: ");
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.println(gps.time.second());
    }

    Serial.println("---------------------------------------------");
  }

  // Alerta si pasan 5 segundos sin recibir datos en absoluto (problema de cableado)
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("ERROR: No se reciben datos del GPS. Verifica el cableado RX/TX.");
    while (true) { delay(500); }
  }
}