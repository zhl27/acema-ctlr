#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

RingbufHandle_t buf_handle;

void setup() {
    Serial.begin(115200);

    // Crear un ring buffer de 1024 bytes de tipo No-Split
    buf_handle = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);

    if (buf_handle == NULL) {
        Serial.println("Error al crear el buffer");
    }
}

void loop() {
    // Enviar datos al buffer
    char miDato[] = "Hola!";
    xRingbufferSend(buf_handle, miDato, sizeof(miDato), pdMS_TO_TICKS(100));

    // Recibir datos del buffer
    size_t item_size;
    char *recibido = (char *)xRingbufferReceive(buf_handle, &item_size, pdMS_TO_TICKS(100));

    if (recibido != NULL) {
        Serial.println(recibido);
        vRingbufferReturnItem(buf_handle, (void *)recibido); // Obligatorio liberar el ítem
    }
    delay(1000);
}