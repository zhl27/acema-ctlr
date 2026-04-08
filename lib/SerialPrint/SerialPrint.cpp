//
// Created by lucaz on 13/2/2026.
//

#include "SerialPrint.h"

constexpr int pow10(int x) {
    return (x == 0) ? 1 : 10 * pow10(x - 1);
}

// tener en cuenta los riesgos de usar variadic functions --> no afecta a nuestro caso de uso
bool SerialPrint::safe_print(const char *format, ...) {
    char buffer[BUFFER_SIZE];
    const unsigned long t_time = millis();
    int res = 0;
    va_list args;
    va_start(args, format);
    res += vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    res += snprintf(buffer, sizeof(buffer), "%s%ct_time%c%ld", buffer, DATA_SEPARATOR, KEYVALUE_SEPARATOR, t_time);

    if (res < 0) {
        Serial.println("ERR$format_error");
        return false;
    }

    if (res >= sizeof(buffer)) {
        Serial.println("ERR$buffer_overflow");
        return false;
    }

    Serial.println(buffer);
    return true;
}


// void SerialPrint::plot(const char key[], const float value) { // TODO: impl defectuosa
//     const int scaled = static_cast<int>(value * pow10(FLOAT_PRECISION));
//     safe_print(
//         "PLOT%c%s%c%d",
//         PROTOCOL_SEPARATOR,
//         key,
//         KEYVALUE_SEPARATOR,
//         FLOAT_PRECISION,
//         scaled
//     );
// }
void SerialPrint::plot(const char key[], const int value) {
    safe_print(
        "PLOT%c%s%c%d",
        PROTOCOL_SEPARATOR,
        key,
        KEYVALUE_SEPARATOR,
        value
    );
}

void SerialPrint::msg(const char* message) {
    safe_print(
        "MSG%c%s",
        PROTOCOL_SEPARATOR,
        message
        // DATA_SEPARATOR
    );
}

void SerialPrint::err(const char* error_msg) {
    safe_print(
        "ERR%c%s",
        PROTOCOL_SEPARATOR,
        error_msg
        // DATA_SEPARATOR
    );
}

/*
 * Versión con buffer dinámico --> genera mem frag, considerar
* static void safePrint(const char* format, ...) {
    char stackBuffer[BUFFER_SIZE];

    va_list args;
    va_start(args, format);

    va_list args_copy;
    va_copy(args_copy, args);

    // Intento en stack
    int len = vsnprintf(stackBuffer, sizeof(stackBuffer), format, args_copy);
    va_end(args_copy);

    if (len < 0) {
        Serial.println("ERR$format_error");
        va_end(args);
        return;
    }

    if (len < sizeof(stackBuffer)) {
        // Cabe → usar stack
        Serial.println(stackBuffer);
        va_end(args);
        return;
    }

    // No cabe → usar heap
    size_t size = len + 1;
    char* heapBuffer = (char*)malloc(size);

    if (!heapBuffer) {
        Serial.println("ERR$malloc_failed");
        va_end(args);
        return;
    }

    vsnprintf(heapBuffer, size, format, args);
    Serial.println(heapBuffer);

    free(heapBuffer);
    va_end(args);
}
 */
