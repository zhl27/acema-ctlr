//
// Created by zhl on 6/27/26.
//

#include "GSE.h"

#include <esp32-hal.h>

LoraWrapped GSE::_lora(LORA_CS, LORA_RST, LORA_DIO0, LORA_DIO1, SPI);

GSE::RocketState GSE::estado_mde() {
    return _currentState;
}

void GSE::init() {
    _currentState = GSE::ROCKET_INIT;
    _previousMillis = 0;
    _cicloContador = 0;
    _simuladorAltitud = 0.0f;
}

void GSE::actualizar() {
    const unsigned long currentMillis = millis();

    switch (_currentState) {

        case ROCKET_INIT:
            if (_lora.begin(DEFAULT_SYNC_WORD, DEFAULT_ENCRY_WORD, DEFAULT_FREC)) {
                Serial.println(F("[COHETE] Hardware LoRa enlazado. Estado: DISCONNECTED"));
                _currentState = ROCKET_DISCONNECTED;
            } else {
                Serial.println(F("[ERROR] No se pudo comunicar con el chip SX1276. Reintentando..."));
                delay(2000); // Retardo seguro únicamente en fase de inicialización crítica
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

                // 1. Simulación circular de variables de ingeniería
                _simuladorAltitud += 2.5f; // Incremento progresivo
                if (_simuladorAltitud > 100.0f) {
                    _simuladorAltitud = 0.0f; // Reseteo circular de 0 a 100
                }

                data_plot_t paqueteTelemetria;
                paqueteTelemetria.altitud = _simuladorAltitud;
                paqueteTelemetria.giroX   = analogRead(A0) * (5.0f / 1023.0f); // Conversión ADC a Voltaje
                paqueteTelemetria.giroY   = 0.0f;  // Variables estáticas de relleno
                paqueteTelemetria.datoX   = _cicloContador;

                // 2. Impresión visual estructurada y cruda antes del envío
                _printear_struct_y_hex(&paqueteTelemetria);

                // 3. Envío mediante método de alto nivel de la fachada
                if (_lora.send_data(paqueteTelemetria)) {
                    Serial.print(F("[TX] Telemetría enviada correctamente. Muestra: "));
                    Serial.println(_cicloContador + 1);
                    _cicloContador++;
                } else {
                    Serial.println(F("[ERROR TX] Pérdida de paquetes o hardware ocupado."));
                }

                // 4. Validación de ciclo de ráfagas (Cada 50 muestras)
                if (_cicloContador % 5 == 0)  {
                    Serial.println(F("\n[EVENTO] Alcanzadas las 50 muestras. Enviando ráfaga de mensajes críticos..."));

                    // Envío de mensaje string común
                    if (_lora.send_msg("HOLA DESDE LA ESTRATOSFERA")) {
                        Serial.println(F("[TX STRING] Mensaje enviado: 'HOLA DESDE LA ESTRATOSFERA'"));
                    }

                    // Envío inmediato de mensaje string de error crítico
                    if (_lora.send_error("Houston, tenemos un problema")) {
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


void GSE::_printear_struct_y_hex(data_plot_t *datos) {
    uint8_t longitudTotal = sizeof(data_plot_t);
    uint8_t protocolo = lora_protocol::C_PLOT;

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
    uint8_t* bytePointer = reinterpret_cast<uint8_t *>(datos);
    for (size_t i = 0; i < longitudTotal; i++) {
        if (bytePointer[i] < 16) Serial.print("0");
        Serial.print(bytePointer[i], HEX);
        Serial.print(" ");
    }
    Serial.println(F("\n--------------------------------------------------"));
}

int GSE::enviar_error(const char* error) {
    return _lora.send_error(error);
}

int GSE::enviar_mensaje(const char* mensaje) {
    return _lora.send_msg(mensaje);
}

int GSE::actualizar_graficas(const data_all_t *data) {
    return _lora.send_data(*data);
}