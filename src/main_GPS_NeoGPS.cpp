#include <Arduino.h>
#include <NMEAGPS.h>       // Off-the-shelf NeoGPS Library
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class HardwareSerial;
// 1. Create a FreeRTOS Queue handle to hold GPS fixes securely
static QueueHandle_t gpsQueue = nullptr;

// 2. Instantiating the off-the-shelf NeoGPS parser
static NMEAGPS gpsParser;
static HardwareSerial& gpsSerial = Serial2;

// --- TAREA DE LECTURA DE GPS (Baja prioridad, corre de fondo) ---
void gpsReceiveTask(void* parameter) {
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // Pines RX=16, TX=17

    while (true) {
        // gpsParser.available() maneja toda la FSM interna byte por byte
        while (gpsParser.available(gpsSerial)) {
            // Obtener el paquete de datos ya resuelto por NeoGPS
            gps_fix validFix = gpsParser.read();

            // Si el fix tiene ubicación válida, lo enviamos de forma segura por la cola
            if (validFix.valid.location) {
                // xQueueOverwrite reemplaza el último dato en la cola de forma atómica.
                // ¡CERO MUTEXES QUE ADMINISTRAR!
                xQueueOverwrite(gpsQueue, &validFix);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // ceder tiempo a otras tareas
    }
}

void setup() {
    Serial.begin(115200);

    // Inicializar la cola de FreeRTOS para almacenar EXACTAMENTE 1 objeto gps_fix
    gpsQueue = xQueueCreate(1, sizeof(gps_fix));

    if (gpsQueue == nullptr) {
        Serial.println("[FATAL] Error creando cola GPS. Abortando.");
        while(1) { vTaskDelay(100); }
    }

    // Lanzar la tarea de lectura en el Core 0
    xTaskCreatePinnedToCore(
        gpsReceiveTask, "GPSTask", 2048, nullptr, 2, nullptr, 0
    );
}

// --- TU LAZO DE CONTROL AEROESPACIAL (150 Hz en Core 1) ---
void loop() {
    static gps_fix currentFlightData; // Almacenamiento local seguro

    // Consultamos si hay un nuevo fix en la cola sin bloquearnos (0 ms de espera)
    // FreeRTOS copia los datos atómicamente. Si no hay datos nuevos, mantiene el último conocido.
    if (xQueueReceive(gpsQueue, &currentFlightData, 0) == pdTRUE) {
        // ¡Datos nuevos y 100% íntegros listos para usar!
    }

    // Consumo súper seguro de los datos procesados por NeoGPS:
    if (currentFlightData.valid.location) {
        // NeoGPS usa enteros de alta precisión (lat/lon * 10^7) por seguridad
        int32_t lat_int = currentFlightData.latitudeL();
        int32_t lon_int = currentFlightData.longitudeL();

        // O si prefieres flotantes estándar:
        float lat_float = currentFlightData.latitude();

        // Altura y velocidad del módulo
        if (currentFlightData.valid.altitude) {
            float alt_meters = currentFlightData.altitude();
        }
    }

    // Tu lazo corre rápido y sin bloqueos de Mutex
    vTaskDelay(pdMS_TO_TICKS(7)); // ~142 Hz
}