//
// Created by lucaz on 13/2/2026.
//

#ifndef ACEMA_CTLR_SERIALPRINT_H
#define ACEMA_CTLR_SERIALPRINT_H

#include <Arduino.h>

#define BUFFER_SIZE 150

// cuidado con que los mensajes contengan los caracteres "$" y ";"
#define PROTOCOL_SEPARATOR '$'
#define DATA_SEPARATOR ';'
#define KEYVALUE_SEPARATOR '='
#define FLOAT_PRECISION 3


// TODO: enviar todos los datos juntos al finalizar un loop (actually un solo loop sigue siendo ineficiente), no inmediatamente, así ahorramos ancho de banda

// TODO: mejorar código para evitar ineficiencia conversión float a string

char* concat(int num, ...);

class SerialPrint {
    static bool safe_print(const char *format, ...);
public:
    static void init(int baudrate);
    // Datos que queremos graficar
    // template <typename T> static void plot(const char key[], T value, ) { // no recomendable en embedded
    static void plot(const char key[], int value);

    // Mensajes triviales. Ej.: "¡Enviado exitosamente!"
    static void msg(const char* message);

    // Mensajes de error
    static void err(const char* error_msg);
};


#endif //ACEMA_CTLR_SERIALPRINT_H
