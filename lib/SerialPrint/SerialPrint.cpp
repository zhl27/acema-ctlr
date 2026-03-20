//
// Created by lucaz on 13/2/2026.
//

#include "SerialPrint.h"

bool SerialPrint::safePrint(const char* format, ...) {
    char buffer[BUFFER_SIZE];

    va_list args;
    va_start(args, format);
    const int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        Serial.println("ERR$format_error");
        return false;
    }

    if (len >= sizeof(buffer)) {
        Serial.println("ERR$buffer_overflow");
        return false;
    }

    Serial.println(buffer);
    return true;
}


void SerialPrint::plot(const char key[], const float value) {
    safePrint(
       "PLOT%c%s%c%.*f",
       PROTOCOL_SEPARATOR,
       key,
       KEYVALUE_SEPARATOR,
       FLOAT_PRECISION,
       value
       // DATA_SEPARATOR
       );
}

void SerialPrint::msg(const char* message) {
    safePrint(
       "MSG%c%s",
       PROTOCOL_SEPARATOR,
       message
       // DATA_SEPARATOR
       );
}

void SerialPrint::err(const char* error_msg) {
    safePrint(
       "ERR%c%s",
       PROTOCOL_SEPARATOR,
       error_msg
       // DATA_SEPARATOR
       );
}