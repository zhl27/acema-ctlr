#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#include <utils/SerialPrint.h>
#include "globals.h"


RingbufHandle_t buf_handle;
char buf[1024];

static uint8_t rx_buffer_storage[BUFFER_SIZE_BYTES];
static StaticRingbuffer_t rx_buffer_struct;

void setup() {
    Serial.begin(115200);

    while (!Serial)
        delay(10); // will pause mcu until serial console opens

    SerialPrint::msg("Setup");

    buf_handle = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);

    if (buf_handle == NULL) {
        SerialPrint::err("Error al crear el buffer");
    }

    delay(100);
}



void loop() { // nuestro main.cpp será una máquina de estados.
    SerialPrint::msg("Loop");
    delay(1000);
}