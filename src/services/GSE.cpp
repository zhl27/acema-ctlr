//
// Created by zhl on 6/27/26.
//

#include "GSE.h"

#include <esp32-hal.h>

#include "SerialPrint.h"

LoraWrapped GSE::_lora(LORA_CS, LORA_RST, LORA_DIO0, LORA_DIO1, SPI);
EstadoConexionGSE GSE::_currentState = EstadoConexionGSE::ROCKET_INIT;             // Assuming an int or an enum
unsigned long GSE::_previousMillis = 0; // Standard type for millis()
int GSE::_cicloContador = 0;
float GSE::_simuladorAltitud = 0.0f;


void GSE::init() {
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
    if(_lora.begin()) {
        SerialPrint::msg("LoRa listo para el Cohete!");
    } else {
        SerialPrint::err("Falla crítica en hardware LoRa");
    }
}

void GSE::actualizar(data_all_t *data) {
    const unsigned long currentMillis = millis();

    switch (_currentState) {

        case ROCKET_INIT:
            if (_lora.begin(DEFAULT_SYNC_WORD, DEFAULT_ENCRY_WORD, DEFAULT_FREC)) {
                Serial.println(F("[COHETE] Hardware LoRa enlazado. Estado: DISCONNECTED"));
                _currentState = ROCKET_DISCONNECTED;
            } else {
                Serial.println(F("[ERROR] No se pudo comunicar con el chip SX1276. Reintentando..."));
                vTaskDelay(pdMS_TO_TICKS(2000)); // Retardo seguro únicamente en fase de inicialización crítica
            }
            break;

        case ROCKET_DISCONNECTED:
            Serial.println(F("[COHETE] Enviando PING de conexión hacia el GSE..."));
            if (_lora.c_connect_to_GSE()) {
                _previousMillis = currentMillis; // Reseteamos temporizador para esperar el PONG
                _currentState = ROCKET_WAITING_PONG;
            }
            break;

        case ROCKET_WAITING_PONG:
            // Escucha no bloqueante del PONG
            if (_lora.c_connection_accepted()) {
                Serial.println(F("[COHETE] ¡PONG Recibido! Enlace confirmado. Estado: CONNECTED"));
                _cicloContador = 0;
                _currentState = ROCKET_CONNECTED;
            }
            // Time-out de reintento: si pasan 3 segundos sin respuesta, vuelve a intentar conectar
            else if (currentMillis - _previousMillis >= 3000) {
                Serial.println(F("[WARN] Tiempo de espera de PONG agotado. Reintentando enlace..."));
                _currentState = ROCKET_DISCONNECTED;
            }
            break;

        case ROCKET_CONNECTED:
            // Evento temporizado cada INTERVALO_TELEMETRIA (s)
            if (currentMillis - _previousMillis >= INTERVALO_TELEMETRIA) {
                _previousMillis = currentMillis;

                _simuladorAltitud += 2.5f; // Incremento progresivo
                if (_simuladorAltitud > 100.0f) {
                    _simuladorAltitud = 0.0f; // Reseteo circular de 0 a 100
                }

                print_data(data); // como es inline, no hay problemas con "Serial"

                if (actualizar_graficas(data)) {
                    Serial.print(F("[TX] Telemetría enviada correctamente. Muestra: "));
                    Serial.println(_cicloContador + 1);
                    _cicloContador++;
                } else {
                    Serial.println(F("[ERROR TX] Pérdida de paquetes o hardware ocupado."));
                }

                if (_cicloContador % 5 == 0)  {
                    Serial.println(F("\n[EVENTO] Alcanzadas las 50 muestras. Enviando ráfaga de mensajes críticos..."));

                    if (enviar_mensaje("HOLA DESDE LA ESTRATOSFERA")) {
                        Serial.println(F("[TX STRING] Mensaje enviado: 'HOLA DESDE LA ESTRATOSFERA'"));
                    }

                    if (enviar_error("Houston, tenemos un problema")) {
                        Serial.println(F("[TX ERROR] Mensaje crítico enviado: 'Houston, tenemos un problema'"));
                    }

                    // Resetear contador para iniciar el siguiente ciclo de 50 telemetrías
                    // cicloContador = 0;
                    // Serial.println(F("[MDE] Reiniciando cuenta de ciclo de telemetría.\n"));
                }
            }
            break;
    }
}

bool GSE::enviar_error(const char* error) {
    return _lora.send_error(error);
}

bool GSE::enviar_mensaje(const char* mensaje) {
    return _lora.send_msg(mensaje);
}

bool GSE::actualizar_graficas(const data_all_t *data) {
    return _lora.send_data(*data);
}