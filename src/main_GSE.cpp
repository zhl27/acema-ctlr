#include <Arduino.h>
#include <LoraWrapped.h>
#include "main.h"
// ============================================================================
// CONFIGURACIÓN DE DEPURACIÓN Y HARDWARE
// ============================================================================
#define DEBUG // Comentar esta línea para desactivar logs de texto y activar salida binaria GUI

// Definición de los pines físicos del ESP32 conectados al módulo LoRa
#define ESP32_LORA_SCK  18
#define ESP32_LORA_MISO 19
#define ESP32_LORA_MOSI 23
#define ESP32_LORA_CS   5
#define ESP32_LORA_RST  14
#define ESP32_LORA_DIO0 2
#define ESP32_LORA_DIO1 4

// Instancia de la clase envoltorio
LoraWrapped lora(ESP32_LORA_CS, ESP32_LORA_RST, ESP32_LORA_DIO0, ESP32_LORA_DIO1, SPI);

// Estados del GSE
enum GseState : uint8_t {
    GSE_INIT,
    GSE_WAITING_CONNECTION,
    GSE_RECEIVING_DATA
};

GseState currentState = GSE_INIT;

// ============================================================================
// ESTRUCTURAS DE DATOS ESPERADAS
// ============================================================================

// PROVICIONAL: Estructura adaptada para los datos que enviará el cohete
// LA ESTRUCTURA DE LOS DATOS A RECIBIR data_gse_t PERO AHORA ESTAMOS USANDO data_all_t



// ELIMINAR TODAS LAS ESTRUCTURAS DE ARRIBA, DEBEN EXITIR EN ALGUN LADO DEL PROYECTO
// ============================================================================
// FUNCIONES AUXILIARES DE ENVÍO (GSE -> PC)
// ============================================================================
#ifndef DEBUG
// Envía los datos hacia la interfaz Qt6 en formato: <'$'><dato1><','><dato2><'#'>
void sendTelemetryToGUI(pkt_t* pkt) {
    if (pkt->protocol == lora_protocol::C_PLOT) {
        data_gse_t* datos = (data_gse_t*)pkt->payload;
        // COMPLETAR CON LOS DEMAS DATOS DE data_all_t
        Serial.print('$');
        Serial.print(datos->vel_angular_x_deg_s, 2); Serial.print(',');
        Serial.print(datos->vel_angular_y_deg_s, 2); Serial.print(',');
        Serial.print(datos->vel_angular_z_deg_s, 2); Serial.print(',');
        Serial.print(datos->temperatura_amb_c, 2);
        Serial.print('#');
        // Si el parser en Qt6 necesita un salto de línea al final de la trama, 
        // descomenta la siguiente línea:
        Serial.println(); 
    } 
	else if (pkt->protocol == lora_protocol::C_ACK) {
		CmdResult* ack = (CmdResult*)pkt->payload;
		Serial.print("$ACK,");
		Serial.print(ack->status); Serial.print(',');
		Serial.print(ack->data, 2);
		Serial.print('#');
		Serial.println(); // Salto de línea para que el parser de la GUI lea la trama completa
	}
}
#endif

#ifdef DEBUG
int CONTADOR = 1;

// Imprime la estructura completa del paquete y su volcado
void printPacketDebug(pkt_t* pkt) {
    Serial.println(F("\n=================================================="));
    Serial.print(F("[INFO] Paquete Recibido | Len: ")); Serial.print(pkt->len);
    Serial.print(F(" | Protocolo: 0x")); Serial.println(pkt->protocol, HEX);
    
    if(pkt->protocol == lora_protocol::PING){
        lora.send_pong();
        Serial.println(F("  [>> PONG ENVIADO]"));
    }
    else if (pkt->protocol == lora_protocol::C_PLOT) {
        data_all_t* datos = (data_all_t*)pkt->payload;
        CONTADOR = (int)datos->vel_angular_x_deg_s;

        Serial.println(F("--- DATOS DE TELEMETRÍA (PLOT) ---"));
        Serial.print(F("  vel angular x:   ")); Serial.println(datos->vel_angular_x_deg_s);
        Serial.print(F("  vel angular y:   ")); Serial.println(datos->vel_angular_y_deg_s);
        Serial.print(F("  vel angular z:   ")); Serial.println(datos->vel_angular_z_deg_s);
        Serial.print(F("  temperatura amb c: ")); Serial.println(datos->temperatura_amb_c);
    } 
    else if (pkt->protocol == lora_protocol::C_MGS || pkt->protocol == lora_protocol::C_ERR) {
        Serial.print(F("  Mensaje String: ")); Serial.println((char*)pkt->payload);
    }
    else if (pkt->protocol == lora_protocol::C_ACK) {
		CmdResult* ack = (CmdResult*)pkt->payload;
		Serial.println(F("--- ACK DE COMANDO RECIBIDO ---"));
		// Traducimos el status en un mensaje legible (Ej. Estado de continuidad devuelto por el pyro)
		Serial.print(F("  Status de Ejecución: ")); Serial.println(ack->status == 1 ? "EXITO / CONTINUIDAD OK (1)" : "ERROR / SIN CONTINUIDAD (0)");
		Serial.print(F("  Dato devuelto:       ")); Serial.println(ack->data);
	}
    
    // Volcado Hexadecimal del crudo
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
// TAREA FREERTOS: RECEPCIÓN DE COMANDOS (PC -> GSE)
// ============================================================================
void vTaskSerialCommands(void *pvParameters) {
    // Timeout para la lectura de bytes en modo GUI para no bloquear infinitamente
    Serial.setTimeout(100); 

    for (;;) {
        if (Serial.available() > 0) {
            #ifndef DEBUG
            // ----------------------------------------------------------------
            // MODO RELEASE: Parseo de trama binaria de la GUI <#><8 bytes><$>
            // ----------------------------------------------------------------
            if (Serial.read() == '#') {
                uint8_t buffer[8];
                size_t readBytes = Serial.readBytes(buffer, 8);
                // TODAVIA NO HAY UNA LISTA OFICIAL DE COMANDO, PERO SE ESTÁN DESARROLLANDO EN config.h
				// TODO: HACER UNA LISTA EN EL DRIVE, PARA TENER A MANO
                if (readBytes == 8) {
                    if (Serial.read() == '$') {
                        uint32_t opCode;
                        float value;
                        // Extraemos los 8 bytes: 4 para el código, 4 para el valor
                        memcpy(&opCode, buffer, sizeof(uint32_t));
                        memcpy(&value, buffer + sizeof(uint32_t), sizeof(float));
                        
                        
                        lora.g_send_command(opCode, value);
                    }
                }
            }
            // Dentro de vTaskSerialCommands(void *pvParameters)
            #endif
			#ifdef DEBUG
			// ----------------------------------------------------------------
			// MODO DEBUG: Parseo de comandos por terminal "nombreComando:valor"
			// ----------------------------------------------------------------
			String input = Serial.readStringUntil('\n');
			input.trim(); // Limpia caracteres ocultos (\r, espacios)

			if (input.length() > 0) {
				int separatorIdx = input.indexOf(':');
				if (separatorIdx != -1) {
					String cmdStr = input.substring(0, separatorIdx);
					float value = input.substring(separatorIdx + 1).toFloat();
					
					uint32_t opCode = 0;
					
					// Mapeo (Los números deben coincidir con los config.h del cohete)
					if (cmdStr == "CLEAR_LOG") opCode = 1;
					else if (cmdStr == "DUMP_DATA") opCode = 2;
					else if (cmdStr == "COMMIT") opCode = 3;
					else if (cmdStr == "DEPLOY_DROGUE") opCode = 4; // CMD_DESPLEGAR_DROGUE
					else if (cmdStr == "DEPLOY_MAIN") opCode = 5;   // CMD_DESPLEGAR_MAIN
					else if (cmdStr == "SET_SERVO") opCode = 6;     // CMD_SET_SERVO
					
					Serial.print(F("[GSE] Comando recibido de PC -> Enviar a Cohete | OP: ")); 
					Serial.print(opCode); Serial.print(F(" | VALOR: ")); Serial.println(value);
					CommandPayload cmd = {.opCode = opCode, .value = value};
					// Envío por RF usando la librería base
					lora.g_send_command(cmd);
				} else {
					Serial.println(F("[GSE-ERR] Formato de comando inválido. Use 'COMANDO:VALOR'"));
				}
			}
			#endif
        }
        
        // Cede control al RTOS (20ms de refresco es suficiente para un humano/GUI)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ============================================================================
// CONFIGURACIÓN PRINCIPAL
// ============================================================================
void setup() {
    Serial.begin(115200);
    #ifdef DEBUG
    Serial.println(F("[GSE] Iniciando sistema de recepción..."));
    #endif
    
    // Inicializar el bus SPI con sus pines físicos
    SPI.begin(ESP32_LORA_SCK, ESP32_LORA_MISO, ESP32_LORA_MOSI, ESP32_LORA_CS);

    // Lanzar la tarea para escuchar comandos por USB asíncronamente
    // Se ancla al Core 0 para dejar el Core 1 libre para la máquina de estados principal
    xTaskCreatePinnedToCore(
        vTaskSerialCommands, 
        "Task_Serial_Cmds", 
        4096, 
        NULL, 
        1, 
        NULL, 
        0
    );
}

// ============================================================================
// LAZO PRINCIPAL (MÁQUINA DE ESTADOS)
// ============================================================================
void loop() {
    switch (currentState) {
        
        case GSE_INIT:
            if (lora.begin(DEFAULT_SYNC_WORD, DEFAULT_ENCRY_WORD, DEFAULT_FREC)) {
                #ifdef DEBUG
                Serial.println(F("[GSE] Modulo LoRa inicializado con éxito."));
                #endif
                currentState = GSE_RECEIVING_DATA;
            } else {
                #ifdef DEBUG
                Serial.println(F("[ERROR] Falló inicialización de LoRa. Reintentando en 2s..."));
                #endif
                delay(2000);
            }
            break;

        case GSE_WAITING_CONNECTION:
            if (lora.g_accept_connection()) {
                #ifdef DEBUG
                Serial.println(F("[GSE] ¡Conexión establecida con el Cohete! Cambiando a modo escucha."));
                #endif
                currentState = GSE_RECEIVING_DATA;
            }
            break;

        case GSE_RECEIVING_DATA:
            pkt_t paqueteRecibido;
            
            if(lora.read_packet(&paqueteRecibido)){
                #ifndef DEBUG
                sendTelemetryToGUI(&paqueteRecibido); 
                #endif
                #ifdef DEBUG
                printPacketDebug(&paqueteRecibido);
                if (paqueteRecibido.protocol == lora_protocol::C_PLOT) {
                    Serial.println("============================================");
                    Serial.print("MUESTRA NUMERO :");
                    Serial.println(CONTADOR);
                    Serial.println("============================================");
                }
                #endif
            }
            break;
    }
}