//
// Created by zhl on 6/6/26.
//

#ifndef ACEMA_CTLR_CONFIG_H
#define ACEMA_CTLR_CONFIG_H



#pragma once
#include <cstddef> // Para size_t
#include <cstdint>


/************************/
/*       FREERTOS       */
/************************/
// Ya está definida con el mismo valor en rtos
//#define configSUPPORT_STATIC_ALLOCATION 1;


namespace ConfigInit {

    /*************************/
    /*    Buses de Datos     */
    /*************************/
    // Bus I2C_0 (Sensores)
    constexpr int WIRE_SDA_0 =  21; // D21 (Pin físico 11)
    constexpr int WIRE_SCL_0 =  22; // D22 (Pin físico 14)

    // Bus SPI Nativo (LoRa, Flash, SD)
    constexpr uint8_t SPI_SCK   = 18; // D18 (Pin físico 9)
    constexpr uint8_t SPI_MISO  = 19; // D19 (Pin físico 10)
    constexpr uint8_t SPI_MOSI  = 23; // D23 (Pin físico 15)

    // Bus UART GPS
    constexpr int GPS_RX_PIN =  16; // Conectar al pin TX del módulo NEO-7M
    constexpr int GPS_TX_PIN =  17; // Conectar al pin RX del módulo NEO-7M

    /*************************/
    /*   Chip Selects (CS)   */
    /*************************/
    constexpr uint8_t LORA_CS   = 5;  // D5  (Pin físico 8  -> NSS_LoRa)
    constexpr uint8_t FLASH_CS  = 4;  // D4  (Pin físico 5  -> CS_FLASH)
    constexpr uint8_t SD_CS     = 15; // D15 (Pin físico 3  -> CS_SD)

    /*************************/
    /*  Pines de Control RF  */
    /*************************/
    constexpr uint8_t LORA_DIO0 = 2;  // D2  (Pin físico 4  -> DIO0 / Rx-Tx Done)
    
    // Nota: Si el Reset del LoRa está atado al Reset general de la ESP32 por hardware,
    // o a un pin específico, definilo acá. Si no se usa, se puede pasar como "NC".
    constexpr uint8_t LORA_RST  = 14;

    // Usamos el equivalente numérico de RADIOLIB_NC (uint32_t max)
    // para indicar que eléctricamente no está conectado a ningún GPIO.
    constexpr uint32_t LORA_DIO1 = 0xFFFFFFFF;
    /*************************/
    /*         GPIOs         */
    /*************************/
    constexpr int BUZZER_PIN =  25;
    
    constexpr int SERVO_PIN  =  27;  
    

    // TODO: CAMBIAR LOS PINES POR LOS REALES
    constexpr uint8_t PIRO_PRINCIPAL_PIN                = 13;
    constexpr uint8_t CONTINUIDAD_PIRO_PRINCIPAL_PIN    = 39;

    constexpr uint8_t PIRO_DROGUE_PIN                   = 12;
    constexpr uint8_t CONTINUIDAD_PIRO_DROGUE_PIN       = 39;
    

    /*************************/
    /*    DIRECCIONES I2C    */
    /*************************/
    constexpr uint8_t MPU_ADDR       = 0x69;
    constexpr uint8_t BMP280_ADDR    = 0x77;

    /***********************/
    /*         MdE         */
    /***********************/
    constexpr int   UMBRAL_LANZAMIENTO      = 100; // TODO: falta implementar valor real
    constexpr float AREA_REFERENCIA_COHETE  = 0.01; // m^2, TODO: falta implementar valor real

    /***********************/
    /*         UART        */
    /***********************/
    constexpr int SERIAL_BAUDRATE_LOG = 115200; // Referido al puerto serial por PC (no gps)



    /**********************/
    /*        MAIN        */
    /**********************/

    constexpr int BUFFER_SIZE_BYTES = 1024*5;


    /**********************/
    /*    PIROTECNICOS    */
    /**********************/
    constexpr int UMBRAL_MIN_CONTINUIDAD_PIRO_mV = 900;

    /**********************/
    /*   FILTROS KALMAN   */
    /**********************/
    constexpr float VARIANZA_INICIAL_GIROSCOPIO     =   0.0048f; // (4°)^2
    constexpr float VARIANZA_INICIAL_ACELEROMETRO   =   0.0027f; // (2°)^2

    /**********************/
    /*     FILTROS EMA    */
    /**********************/
    constexpr float EMA_FREC_CORTE_TEMPERATURA     = 1.0f;
    constexpr float EMA_FREC_CORTE_PRESION_ATM     = 2.0f;
    constexpr float EMA_FREC_CORTE_DENSIDAD_AIRE   = 1.0f;
    constexpr float EMA_FREC_CORTE_ACCEL_VERTICAL  = 15.0f;

    /**********************/
    /*     BUFFERS TAM    */
    /**********************/
    // constexpr size_t RBUF_SIZE = 8192; // bytes per ring buffer (Anterior)
    constexpr std::size_t RBUF_SIZE             = 4096; 
    constexpr std::size_t BUF_Q_SENSOR_SIZE     = 1024;

    /**********************/
    /*     FRECUENCIAS    */
    /**********************/
    constexpr float PERIOD_SAMPLING_SENSORS_MS = 100.0f; // Muestreo cada 10 ms
    constexpr float FREC_SAMPLING_SENSORS_HZ = 1.0f / (PERIOD_SAMPLING_SENSORS_MS * 1e-3f);
}

enum comandosGlobale {
    CMD_CLEAR_LOG = 1,
    CMD_DUMP_DATA,
    CMD_COMMIT,
    CMD_DESPLEGAR_DROGUE,
    CMD_DESPLEGAR_MAIN,
    CMD_SET_SERVO
};

#endif //ACEMA_CTLR_CONFIG_H
/**
 *  Nota de cambios: 
 * - Se modigicó Sensors.cpp
 *  - Se cambió Actuators.cpp*/