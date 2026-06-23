#include <Arduino.h>
#include <LoraWrapped.h>

// ============================================================================
// CONFIGURACIÓN CONDICIONAL DE PINES SEGÚN EL ENTORNO DEL .INI
// ============================================================================

#if defined(MICRO_ESP32)
    // Pines asignados si compilas con: pio run -e CPU-esp32
    #define LORA_SCK  18
    #define LORA_MISO 19
    #define LORA_MOSI 23
    #define LORA_CS   5
    #define LORA_RST  14
    #define LORA_DIO0 2
    #define LORA_DIO1 4
// Instanciación única y genérica usando los alias de los macros
LoraWrapped lora(LORA_CS, LORA_RST, LORA_DIO0, LORA_DIO1, SPI);

#elif defined(MICRO_NANO)
    // Pines asignados si compilas con: pio run -e CPU-nano
    #define LORA_CS   10  // Va a NSS (con divisor)
    #define LORA_RST  9   // Va a RST (con divisor)
    #define LORA_DIO1 2   // Va a DIO1 (directo)
    #define LORA_BUSY RADIOLIB_NC   // Va a BUSY (directo)
// El orden de los argumentos debe coincidir con tu constructor modificado:
LoraWrapped lora(LORA_CS, LORA_RST, LORA_DIO1, LORA_BUSY, SPI);
#endif



// Estados de la MDE del Cohete
enum RocketState : uint8_t {
    ROCKET_INIT,
    ROCKET_DISCONNECTED,
    ROCKET_WAITING_PONG,
    ROCKET_CONNECTED
};

RocketState currentState = ROCKET_INIT;

// Variables de control de tiempo y ciclos
unsigned long previousMillis = 0;
const long INTERVALO_TELEMETRIA = 5000; // 5s de frecuencia de envío

int cicloContador = 0;
float simuladorAltitud = 0.0f;

// ============================================================================
// FUNCIÓN DE IMPRESIÓN ESTRUCTURAL Y HEXADECIMAL (CRUDO)
// ============================================================================
void mostrarEstructuraYHex(dataPlot_t* datos) {
    uint8_t longitudTotal = sizeof(dataPlot_t);
    uint8_t protocolo = Protocolo::C_PLOT;

    Serial.println(F("\n--------------------------------------------------"));
    Serial.println(F("--- ESTRUCTURA VISUAL DEL PAQUETE A ENVIAR ---"));
    Serial.print(F(" Longitud del Payload (len): ")); Serial.print(longitudTotal); Serial.println(F(" bytes"));
    Serial.print(F(" Identificador Protocolo:     0x0")); Serial.println(protocolo, HEX);
    Serial.println(F(" Datos Internos Estructura:"));
    Serial.print(F("   Altitud: ")); Serial.println(datos->altitud);
    Serial.print(F("   GiroX:   ")); Serial.println(datos->giroX);
    Serial.print(F("   GiroY:   ")); Serial.println(datos->giroY);
    Serial.print(F("   DatoX:   ")); Serial.println(datos->datoX);
    
    // Imprimir el paquete tal como viajará en crudo por el aire [len][proto][payload]
    Serial.print(F("PAQUETE CRUDO (HEX): "));
    
    // 1. Byte de longitud
    if (longitudTotal < 16) {
        Serial.print("0"); Serial.print(longitudTotal, HEX); Serial.print(" ");
    }
    // 2. Byte de protocolo
    if (protocolo < 16) {
        Serial.print("0"); Serial.print(protocolo, HEX); Serial.print(" ");
    }
    // 3. Bytes del payload struct
    uint8_t* bytePointer = (uint8_t*)datos;
    for (size_t i = 0; i < longitudTotal; i++) {
        if (bytePointer[i] < 16) Serial.print("0");
        Serial.print(bytePointer[i], HEX);
        Serial.print(" ");
    }
    Serial.println(F("\n--------------------------------------------------"));
}

// ============================================================================
// CONFIGURACIÓN PRINCIPAL
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("[COHETE] Sistema inicializado de telemetría."));

// Inicialización del bus SPI condicional
    #if defined(MICRO_ESP32)
        // El ESP32 mapea el SPI por software en las patas elegidas
        SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
        Serial.println("Inicializando SPI en modo ESP32...");
    #elif defined(MICRO_NANO)
        // El Nano usa sus pines fijos de hardware por defecto
// 1. Configurar Chip Select
        pinMode(LORA_CS, OUTPUT);
        digitalWrite(LORA_CS, HIGH); 

        // 2. Configurar el pin de Reset explícitamente desde el main para asegurar el arranque
        pinMode(LORA_RST, OUTPUT);
        digitalWrite(LORA_RST, LOW);    // Forzamos el reset físico (0V)
        delay(20);                      // Mantenemos el reset 20ms
        digitalWrite(LORA_RST, HIGH);   // Liberamos el reset (Sube a 3.2V)
        delay(50);                      // CRUCIAL: Esperamos 50ms a que el SX1262 inicialice su firmware interno

        // 3. Arrancar el SPI nativo a baja velocidad para los divisores
        SPI.begin();
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

        Serial.println("Inicializando SPI en modo Arduino Nano...");
        
    #endif

    if(lora.begin()) {
        Serial.println("LoRa listo para el Cohete!");
    } else {
        Serial.println("Falla crítica en hardware LoRa");
    }
}

// ============================================================================
// LAZO PRINCIPAL
// ============================================================================


void loop() {
    unsigned long currentMillis = millis();

    switch (currentState) {

        case ROCKET_INIT:
            if (lora.begin(DEFAULT_SYNC_WORD, DEFAULT_ENCRY_WORD, DEFAULT_FREC)) {
                Serial.println(F("[COHETE] Hardware LoRa enlazado. Estado: DISCONNECTED"));
                currentState = ROCKET_DISCONNECTED;
            } else {
                Serial.println(F("[ERROR] No se pudo comunicar con el chip SX1276. Reintentando..."));
                delay(2000); // Retardo seguro únicamente en fase de inicialización crítica
            }
            break;

        case ROCKET_DISCONNECTED:
            Serial.println(F("[COHETE] Enviando PING de conexión hacia el GSE..."));
            if (lora.c_connect_to_GSE()) {
                previousMillis = currentMillis; // Reseteamos temporizador para esperar el PONG
                currentState = ROCKET_WAITING_PONG;
            }
            break;

        case ROCKET_WAITING_PONG:
            // Escucha no bloqueante del PONG
            if (lora.c_connection_accepted()) {
                Serial.println(F("[COHETE] ¡PONG Recibido! Enlace confirmado. Estado: CONNECTED"));
                cicloContador = 0;
                currentState = ROCKET_CONNECTED;
            } 
            // Time-out de reintento: si pasan 3 segundos sin respuesta, vuelve a intentar conectar
            else if (currentMillis - previousMillis >= 3000) {
                Serial.println(F("[WARN] Tiempo de espera de PONG agotado. Reintentando enlace..."));
                currentState = ROCKET_DISCONNECTED;
            }
            break;

        case ROCKET_CONNECTED:
            // Evento temporizado cada INTERVALO_TELEMETRIA (s)
            if (currentMillis - previousMillis >= INTERVALO_TELEMETRIA) {
                previousMillis = currentMillis;

                // 1. Simulación circular de variables de ingeniería
                simuladorAltitud += 2.5f; // Incremento progresivo
                if (simuladorAltitud > 100.0f) {
                    simuladorAltitud = 0.0f; // Reseteo circular de 0 a 100
                }

                dataPlot_t paqueteTelemetria;
                paqueteTelemetria.altitud = simuladorAltitud;
                paqueteTelemetria.giroX   = analogRead(A0) * (5.0f / 1023.0f); // Conversión ADC a Voltaje
                paqueteTelemetria.giroY   = 0.0f;  // Variables estáticas de relleno
                paqueteTelemetria.datoX   = cicloContador;

                // 2. Impresión visual estructurada y cruda antes del envío
                mostrarEstructuraYHex(&paqueteTelemetria);

                // 3. Envío mediante método de alto nivel de la fachada
                if (lora.send_datos(paqueteTelemetria)) {
                    Serial.print(F("[TX] Telemetría enviada correctamente. Muestra: "));
                    Serial.println(cicloContador + 1);
                    cicloContador++;
                } else {
                    Serial.println(F("[ERROR TX] Pérdida de paquetes o hardware ocupado."));
                }

                // 4. Validación de ciclo de ráfagas (Cada 50 muestras)
                if (cicloContador >= 5) {
                    Serial.println(F("\n[EVENTO] Alcanzadas las 50 muestras. Enviando ráfaga de mensajes críticos..."));

                    // Envío de mensaje string común
                    if (lora.send_mensaje("HOLA DESDE LA ESTRATOSFERA")) {
                        Serial.println(F("[TX STRING] Mensaje enviado: 'HOLA DESDE LA ESTRATOSFERA'"));
                    }

                    // Envío inmediato de mensaje string de error crítico
                    if (lora.send_mensaje_error("Houston, tenemos un problema")) {
                        Serial.println(F("[TX ERROR] Mensaje crítico enviado: 'Houston, tenemos un problema'"));
                    }

                    // Resetear contador para iniciar el siguiente ciclo de 50 telemetrías
                    cicloContador = 0;
                    Serial.println(F("[MDE] Reiniciando cuenta de ciclo de telemetría.\n"));
                }
            }
            break;
    }
}