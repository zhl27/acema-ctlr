#include <Arduino.h>
#include <LoraWrapped.h>

// ============================================================================
// CONFIGURACIÓN DE DEPURACIÓN Y HARDWARE
// ============================================================================
#define DEBUG // Comentar esta línea para desactivar logs de texto y activar salida binaria GUI


// Definís los pines específicos que ruteaste en la PCB de tu ESP32

#ifdef XL1262_p01


#endif

#ifndef XL1262_p01

#define ESP32_LORA_SCK  18
#define ESP32_LORA_MISO 19
#define ESP32_LORA_MOSI 23
#define ESP32_LORA_CS   5
#define ESP32_LORA_RST  14
#define ESP32_LORA_DIO0 2
#define ESP32_LORA_DIO1 4

#endif


// Instanciamos pasándole los pines correspondientes
LoraWrapped lora(ESP32_LORA_CS, ESP32_LORA_RST, ESP32_LORA_DIO0, ESP32_LORA_DIO1, SPI);

// Estados del GSE
enum GseState : uint8_t {
    GSE_INIT,
    GSE_WAITING_CONNECTION,
    GSE_RECEIVING_DATA
};

GseState currentState = GSE_INIT;

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================
#ifndef DEBUG
// Envía el paquete crudo por Serial hacia la interfaz Qt6 en formato: '$'<Bytes>'#'
void sendBinaryToGUI(pkt_t* pkt) {
    Serial.write('$');
    Serial.write(pkt->len);
    Serial.write(pkt->protocol);
    for (int i = 0; i < pkt->len; i++) {
        Serial.write(((uint8_t*)pkt->payload)[i]);
    }
    Serial.write('#');
}
#endif

#ifdef DEBUG

int CONTADOR = 1;

// Imprime la estructura completa del paquete y su volcado en Hexadecimal
void printPacketDebug(pkt_t* pkt) {
    Serial.println(F("\n=================================================="));
    Serial.print(F("[INFO] Paquete Recibido | Len: ")); Serial.print(pkt->len);
    Serial.print(F(" | Protocolo: 0x")); Serial.println(pkt->protocol, HEX);
    
    // Imprimir de acuerdo al tipo de protocolo detectado
    
    if(pkt->protocol == lora_protocol::PING){
        lora.send_pong();
    }
    else if (pkt->protocol == lora_protocol::C_PLOT) {
        data_all_t* datos = (data_all_t*)pkt->payload;

        CONTADOR = datos->vel_angular_x_deg_s;

        Serial.println(F("--- DATOS DE TELEMETRÍA (PLOT) ---"));
        Serial.print(F("  vel angular x:   ")); Serial.println(datos->vel_angular_x_deg_s);
        Serial.print(F("  vel angular y:   ")); Serial.println(datos->vel_angular_y_deg_s);
        Serial.print(F("  vel angular z:   ")); Serial.println(datos->vel_angular_z_deg_s);
        Serial.print(F("  temperatura amb c: ")); Serial.println(datos->temperatura_amb_c);
    } 
    else if (pkt->protocol == lora_protocol::C_MGS || pkt->protocol == lora_protocol::C_ERR) {
        Serial.print(F("  Mensaje String: ")); Serial.println((char*)pkt->payload);
    }
    
    // Volcado Hexadecimal del crudo (len, protocolo y payload)
    Serial.print(F("Crudo HEX: "));
    if (pkt->len < 16) Serial.print("0"); Serial.print(pkt->len, HEX); Serial.print(" ");
    if (pkt->protocol < 16) Serial.print("0"); Serial.print(pkt->protocol, HEX); Serial.print(" ");
    
    for (int i = 0; i < pkt->len; i++) {
        uint8_t b = ((uint8_t*)pkt->payload)[i];
        if (b < 16) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
    }
    Serial.println(F("\n=================================================="));
}
#endif


// ============================================================================
// CONFIGURACIÓN PRINCIPAL
// ============================================================================
void setup() {
    Serial.begin(115200);
    #ifdef DEBUG
    Serial.println(F("[GSE] Iniciando sistema de recepción..."));
    #endif
    // OBLIGATORIO EN ESP32: Inicializar el bus SPI con sus pines físicos
    SPI.begin(ESP32_LORA_SCK, ESP32_LORA_MISO, ESP32_LORA_MOSI, ESP32_LORA_CS);

    
}

// ============================================================================
// LAZO PRINCIPAL (MÁQUINA DE ESTADOS)
// ============================================================================
void loop() {
    switch (currentState) {
        
        case GSE_INIT:
            if (lora.begin(DEFAULT_SYNC_WORD, DEFAULT_ENCRY_WORD, DEFAULT_FREC)) {
                #ifdef DEBUG
                Serial.println(F("[GSE] Modulo LoRa inicializado con éxito. Esperando cohete..."));
                #endif
                currentState = GSE_RECEIVING_DATA;//GSE_WAITING_CONNECTION;
            } else {
                #ifdef DEBUG
                Serial.println(F("[ERROR] Falló inicialización de LoRa. Reintentando en 2s..."));
                #endif
                delay(2000);
            }
            break;

        case GSE_WAITING_CONNECTION:
            // Escucha activa del PING de conexión
            if (lora.g_accept_connection()) {
                #ifdef DEBUG
                Serial.println(F("[GSE] ¡Conexión establecida con el Cohete! Cambiando a modo escucha."));
                #endif
                currentState = GSE_RECEIVING_DATA;
            }
            break;

        case GSE_RECEIVING_DATA:
            pkt_t paqueteRecibido;
            // TODO: COMPLETAR
            
            // Instanciamos el método genérico público de lectura de la fachada
            // Nota: internamente gestiona la asignación y limpieza según protocolo
            // Pasamos un puntero a una estructura externa provisional para capturar los metadatos.
            //if (lora.g_accept_connection() == false) { 
                // g_accept_connection lee paquetes internamente, pero para capturar los datos
                // de telemetría continuos llamamos directamente a la lectura del paquete.
                
                // Hack estructurado: simulamos la llamada a read_package que declaraste privada.
                // Como read_package es privada, una alternativa limpia para que el GSE lea datos genéricos
                // es exponer un método público en tu fachada o usar una estructura dedicada.
                // Dado que tu fachada lee automáticamente en base al búfer privado, implementamos la recepción:
            //}
            if(lora.read_packet(&paqueteRecibido)){
                #ifndef DEBUG
                sendBinaryToGUI(&paqueteRecibido); 
                #endif
                #ifdef DEBUG
                printPacketDebug(&paqueteRecibido);
                Serial.println("============================================");
                Serial.print("MUESTRA NUMERO :");
                Serial.println(CONTADOR);
                Serial.println("============================================");
                #endif
            }
            // Nota de integración: Para poder procesar datos continuos de manera limpia, 
            // asumimos que implementaste o usás un método público de lectura en la fachada, 
            // aquí emulamos la recepción usando los buffers internos expuestos por el comportamiento de la clase.
            break;
    }
}