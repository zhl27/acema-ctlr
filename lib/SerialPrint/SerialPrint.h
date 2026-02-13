//
// Created by lucaz on 13/2/2026.
//

#ifndef SERIALPRINT_H
#define SERIALPRINT_H

#include <Arduino.h>

#define BUFFER_SIZE 150

#define PROTOCOL_SEPARATOR '$'
#define DATA_SEPARATOR ';'
#define KEYVALUE_SEPARATOR '='
#define FLOAT_PRECISION 3

class SerialPrint {
public:
    // Datos que queremos graficar
    // template <typename T> static void plot(const char key[], T value, ) { // no recomendable en embedded
    static void plot(const char key[], const float value) {
        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer),
           "PLOT%c%s%c%.*f",
           PROTOCOL_SEPARATOR,
           key,
           KEYVALUE_SEPARATOR,
           FLOAT_PRECISION,
           value
           // DATA_SEPARATOR
           );

        Serial.println(buffer);
    }
    // Mensajes triviales. Ej.: "¡Enviado exitosamente!"
    static void msg(const char* message) {
        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer),
           "MSG%c%s",
           PROTOCOL_SEPARATOR,
           message
           // DATA_SEPARATOR
           );

        Serial.println(buffer);
    }
    // Mensajes de error
    static void err(const char* error_msg) {
        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer),
           "ERR%c%s",
           PROTOCOL_SEPARATOR,
           error_msg
           // DATA_SEPARATOR
           );

        Serial.println(buffer);
    }
};



#endif //SERIALPRINT_H
