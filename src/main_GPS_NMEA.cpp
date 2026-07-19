#include <Arduino.h>

// Definición de pines según tus requerimientos
#define GPS_RX_PIN 16 // Conectar al pin TX del módulo NEO-7M
#define GPS_TX_PIN 17 // Conectar al pin RX del módulo NEO-7M

// Usamos el puerto Serial2 por hardware del ESP32
HardwareSerial SerialGPS(2);

void setup() {
    // Iniciar puerto serie para el Monitor Serie de la PC
    Serial.begin(115200);
    while (!Serial); // Esperar a que se conecte el monitor serie

    Serial.println("--- Test Básico del Módulo GPS NEO-7M ---");
    Serial.println("Buscando tramas NMEA...");

    // Iniciar comunicación con el NEO-7M (por defecto suele operar a 9600 baudios, 8N1)
    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
    // Si el GPS envía datos, los leemos y los reenviamos al Monitor Serie de la PC
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read();
        Serial.write(c);
    }

    // Si envías algún comando desde la PC, se lo reenviamos al GPS
    while (Serial.available() > 0) {
        SerialGPS.write(Serial.read());
    }
}