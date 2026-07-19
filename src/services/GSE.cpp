//
// Created by zhl on 6/27/26.
//

#include "GSE.h"

#include <esp32-hal.h>

#include "esp_log.h"
#include "config.h"

static const char *TAG_GSE = "CONEXIÓN GSE";

LoraWrapped GSE::_lora(ConfigInit::LORA_CS, ConfigInit::LORA_RST, ConfigInit::LORA_DIO0, ConfigInit::LORA_DIO1, SPI);
EstadoConexionGSE GSE::_currentState = EstadoConexionGSE::ROCKET_INIT;             // Assuming an int or an enum
unsigned long GSE::_previousMillis = 0; // Standard type for millis()
int GSE::_cicloContador = 0;
float GSE::_simuladorAltitud = 0.0f;


void GSE::init() {
#if defined(MICRO_ESP32)
// 1. FORZAR APAGADO de los otros dispositivos del bus SPI
    // Esto evita que la Flash y la SD interfieran en la línea MISO
    pinMode(ConfigInit::FLASH_CS, OUTPUT);
    digitalWrite(ConfigInit::FLASH_CS, HIGH); // HIGH = Desactivada

    pinMode(ConfigInit::SD_CS, OUTPUT);
    digitalWrite(ConfigInit::SD_CS, HIGH);    // HIGH = Desactivada
// Inicializa el bus SPI compartible. El -1 previene el secuestro del CS por hardware.
    SPI.begin(ConfigInit::SPI_SCK, ConfigInit::SPI_MISO, ConfigInit::SPI_MOSI, -1);
    ESP_LOGI(TAG_GSE, "Inicializando SPI compartido en modo ESP32...");
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
    ESP_LOGI(TAG_GSE, "Inicializando SPI en modo Arduino Nano...");
#endif
    if(_lora.begin()) {
        ESP_LOGI(TAG_GSE, "LoRa listo para el Cohete!");
    } else {
        ESP_LOGE(TAG_GSE, "Falla crítica en hardware LoRa");
    }
}

void GSE::actualizar(data_all_t *data) {
    const unsigned long currentMillis = millis();

    switch (_currentState) {

        case ROCKET_INIT:
            if (_lora.begin(DEFAULT_SYNC_WORD, DEFAULT_ENCRY_WORD, DEFAULT_FREC)) {
                ESP_LOGI(TAG_GSE, "[COHETE] Hardware LoRa enlazado. Estado: DISCONNECTED");
                _currentState = ROCKET_DISCONNECTED;
            } else {
                ESP_LOGI(TAG_GSE, "[ERROR] No se pudo comunicar con el chip SX1276. Reintentando...");
                vTaskDelay(pdMS_TO_TICKS(2000)); // Retardo seguro únicamente en fase de inicialización crítica
            }
            break;

        case ROCKET_DISCONNECTED:
            ESP_LOGI(TAG_GSE, "[COHETE] Enviando PING de conexión hacia el GSE...");
            if (_lora.c_connect_to_GSE()) {
                _previousMillis = currentMillis; // Reseteamos temporizador para esperar el PONG
                _currentState = ROCKET_WAITING_PONG;
            }
            break;

        case ROCKET_WAITING_PONG:
            // Escucha no bloqueante del PONG
            if (_lora.c_connection_accepted()) {
                ESP_LOGI(TAG_GSE, "[COHETE] ¡PONG Recibido! Enlace confirmado. Estado: CONNECTED");
                _cicloContador = 0;
                _currentState = ROCKET_CONNECTED;
            }
            // Time-out de reintento: si pasan 3 segundos sin respuesta, vuelve a intentar conectar
            else if (currentMillis - _previousMillis >= 3000) {
                ESP_LOGI(TAG_GSE, "[WARN] Tiempo de espera de PONG agotado. Reintentando enlace...");
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
                    ESP_LOGI(TAG_GSE, "[TX] Telemetría enviada correctamente. Muestra: %d", _cicloContador + 1);
                    _cicloContador++;
                } else {
                    ESP_LOGE(TAG_GSE, "[ERROR TX] Pérdida de paquetes o hardware ocupado.");
                }

                if (_cicloContador % 5 == 0)  {
                    ESP_LOGI(TAG_GSE, "[EVENTO] Alcanzadas las 50 muestras. Enviando ráfaga de mensajes críticos...");

                    if (enviar_mensaje("HOLA DESDE LA ESTRATOSFERA")) {
                        ESP_LOGI(TAG_GSE, "[TX STRING] Mensaje enviado: 'HOLA DESDE LA ESTRATOSFERA'");
                    }

                    if (enviar_error("Houston, tenemos un problema")) {
                        ESP_LOGI(TAG_GSE, "[TX ERROR] Mensaje crítico enviado: 'Houston, tenemos un problema'");
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

bool GSE::leer_paquete(pkt_t* paquete){
    if(paquete == nullptr){
        return false;
    }
    return _lora.read_packet(paquete); 
}