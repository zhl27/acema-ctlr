#include <Arduino.h>

#include "BluetoothSerial.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h> // Librería necesaria para el servo

// --- CONFIGURACIÓN DE PINES ---
const int pinBuzzer = 4;
const int pinBat = 34;
const int pinServo = 13;    // Tu pin del servo (Cable Amarillo)
const float factorBat = 2.0;

BluetoothSerial SerialBT;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
Servo miServo;

uint32_t lastTime = 0;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  // Configuración de Pines
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinBat, INPUT);

  // Configuración del Servo
  miServo.attach(pinServo);
  miServo.write(90); // Asegurar que inicie detenido

  // Configuración ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // --- SECUENCIA DE INICIO (2 pitidos) ---
  digitalWrite(pinBuzzer, HIGH); delay(200);
  digitalWrite(pinBuzzer, LOW);  delay(200);
  digitalWrite(pinBuzzer, HIGH); delay(200);
  digitalWrite(pinBuzzer, LOW);

  SerialBT.begin("ESP32_Cohete_GPS");
  Serial.println("Sistema iniciado. Bluetooth: ESP32_Cohete_GPS");
}

void loop() {
  // 1. Alimentar la librería GPS
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // 2. Acción del Servo y Telemetría cada 5 segundos (para dar tiempo al movimiento)
  if (millis() - lastTime > 5000) {
    lastTime = millis();

    // --- MOVIMIENTO DEL SERVO ---
    Serial.println("Moviendo Servo: Sentido A");
    miServo.write(0);   // Gira a un lado
    delay(1000);        // Durante 1 segundo

    Serial.println("Moviendo Servo: Sentido B");
    miServo.write(180); // Gira al otro lado
    delay(1000);        // Durante 1 segundo

    miServo.write(90);  // Detener servo

    // --- LECTURA DE BATERÍA ---
    long sumaADC = 0;
    for(int i=0; i<10; i++) { sumaADC += analogRead(pinBat); delay(2); }
    float vPin = (sumaADC / 10.0 / 4095.0) * 3.3;
    float vBat = vPin * factorBat;

    int porcentaje = map(vBat * 100, 340, 420, 0, 100);
    porcentaje = constrain(porcentaje, 0, 100);

    // --- FORMATEO DE MENSAJE ---
    String mensaje = "\n--- TELEMETRIA COHETE ---\n";
    mensaje += "Bateria: " + String(vBat, 2) + "V (" + String(porcentaje) + "%)\n";
    mensaje += "Satelites: " + String(gps.satellites.value()) + "\n";

    if (gps.location.isValid()) {
      mensaje += "Lat: " + String(gps.location.lat(), 6) + "\n";
      mensaje += "Lon: " + String(gps.location.lng(), 6) + "\n";
    } else {
      mensaje += "Posicion: BUSCANDO...\n";
    }

    mensaje += "-------------------------\n";

    // Enviar a Serial y Bluetooth
    Serial.print(mensaje);
    if (SerialBT.connected()) {
      SerialBT.print(mensaje);
    }
  }
}
