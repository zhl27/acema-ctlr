#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SerialPrint.h> // Tu librería personalizada

const int BUZZER_PIN = 25;
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // Pines 16 (RX) y 17 (TX)

// --- VARIABLES PARA CONTROL DE CAMBIOS SIGNIFICATIVOS ---
float last_temp = 0.0;
float last_pres = 0.0;
float last_alt  = 0.0;

// Ajustamos los umbrales para captar la variación del soplido con filtro X16.
// La temperatura casi no se mueve, pero la presión y altitud van a saltar.
const float TEMP_THRESHOLD = 0.2;  // Cambios mayores a 0.2 °C
const float PRES_THRESHOLD = 8.0;  // Subido a 8.0 Pa para evitar falsos positivos por el ruido
const float ALT_THRESHOLD  = 0.6;  // Un soplido superará fácilmente los 0.6 metros acumulados
// --------------------------------------------------------

void setup() {
    Serial.begin(115200);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);

    Wire.begin(21, 22);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

    while (!Serial)
        delay(10);

    Serial.println("Adafruit MPU6050 & BMP280 test!");

    if (!mpu.begin(0x69)) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");

    if (!bmp.begin(0x77, BMP280_CHIPID)) {
        Serial.println("Failed to find BMP280 chip");
        while (1) { delay(10); }
    }
    Serial.println("BMP280 Found!");

    // Mantenemos tu configuración original con filtro alto y delay de 500ms
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    // Leer valores iniciales para que la primera comparación sea real
    last_temp = bmp.readTemperature();
    last_pres = bmp.readPressure();
    last_alt  = bmp.readAltitude(1013.25);

    Serial.println("");

    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    delay(100);
}

void loop() {
    // Alimentar constantemente el objeto GPS
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    // Graficar GPS solo cuando los datos se hayan actualizado realmente
    if (gps.satellites.isUpdated() || gps.location.isUpdated())
    {
        SerialPrint::plot("s.num", static_cast<float>(gps.satellites.value()));
        SerialPrint::plot("s.lat", static_cast<float>(gps.location.lat()));
        SerialPrint::plot("s.long", static_cast<float>(gps.location.lng()));
    }

    // Graficar MPU6050 si se detecta movimiento
    if (mpu.getMotionInterruptStatus())
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        SerialPrint::plot("a.X", static_cast<float>(a.acceleration.x));
        SerialPrint::plot("a.y", static_cast<float>(a.acceleration.y));
        SerialPrint::plot("a.z", static_cast<float>(a.acceleration.z));
        SerialPrint::plot("g.x", static_cast<float>(g.gyro.x));
        SerialPrint::plot("g.y", static_cast<float>(g.gyro.y));
        SerialPrint::plot("g.z", static_cast<float>(g.gyro.z));
    }

    // --- LÓGICA DE CONTROL PARA EL BMP280 ---
    float current_temp = bmp.readTemperature();
    float current_pres = bmp.readPressure();
    float current_alt  = bmp.readAltitude(1013.25);

    // Usamos abs() para evaluar la diferencia absoluta (tanto subidas como bajadas)
    bool temp_changed = abs(current_temp - last_temp) >= TEMP_THRESHOLD;
    bool pres_changed = abs(current_pres - last_pres) >= PRES_THRESHOLD;
    bool alt_changed  = abs(current_alt - last_alt)   >= ALT_THRESHOLD;

    // Si cualquiera de los tres parámetros cambia significativamente, entra al condicional
    if (temp_changed || pres_changed || alt_changed)
    {
        // MODIFICACIÓN: Si el cambio fue por presión o altitud, asumimos que soplaste
        if (pres_changed || alt_changed) {
            SerialPrint::msg("¡Soplido detectado en el BMP280!");
        }

        SerialPrint::plot("bmp_temp", static_cast<float>(current_temp));
        SerialPrint::plot("bmp_pres", static_cast<float>(current_pres));
        SerialPrint::plot("bmp_alt",  static_cast<float>(current_alt));

        // Actualizamos el historial con los valores actuales
        last_temp = current_temp;
        last_pres = current_pres;
        last_alt  = current_alt;
    }

    // Delay mínimo para evitar saturar el Core del ESP32 pero permitir respiro al búfer serial
    delay(1);
}