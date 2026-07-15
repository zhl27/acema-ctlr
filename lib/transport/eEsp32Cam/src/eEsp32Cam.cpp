//
// Created by lucaz on 11/7/2026.
//

#include "eEsp32Cam.h"

#include <driver/uart.h>


// uint8_t eEsp32Cam::_calcularChecksum(const char* txt) {
//     uint8_t checksum = 0;
//     for (int i = 0; i < strlen(txt); i++) {
//         checksum ^= txt[i];
//     }
//     return checksum;
// }

uint8_t eEsp32Cam::_calculate_crc8(uint8_t opcode, uint8_t param) {

}



bool eEsp32Cam::sendCommandToCam(uint8_t opcode, uint8_t param) {
    uint8_t tx_packet[4];
    tx_packet[0] = 0xAA;               // Sync byte
    tx_packet[1] = opcode;             // e.g., CMD_CAM_CTRL
    tx_packet[2] = param;              // e.g., 1 (Start)
    tx_packet[3] = _calculate_crc8(tx_packet[1], tx_packet[2]); // Simple CRC

    int retries = 3;
    while (retries > 0) {
        // 1. Send the 4 bytes
        uart_write_bytes(UART_NUM_1, (const char*)tx_packet, 4);

        // 2. Wait for exactly 1 byte (ACK) with a 50ms timeout
        uint8_t rx_ack = 0;
        int len = uart_read_bytes(UART_NUM_1, &rx_ack, 1, pdMS_TO_TICKS(50));

        if (len > 0 && rx_ack == ACK_SUCCESS) {
            return true; // Success! Camera is rolling.
        }

        // If we get here, either timeout or NACK. Try again.
        retries--;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false; // Camera failed to respond after 3 tries
}
