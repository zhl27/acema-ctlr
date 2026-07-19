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

#define configSUPPORT_STATIC_ALLOCATION 1;


namespace ConfigInit {

    /*************************/
    /*         GPIOs         */
    /*************************/
    constexpr int BUZZER_PIN =  25;
    
    constexpr int WIRE_SDA_0 =  21;
    constexpr int WIRE_SCL_0 =  22;
    
    constexpr int SERVO_PIN  =  27;  
    
    constexpr int GPS_RX_PIN =  16; // Conectar al pin TX del módulo NEO-7M
    constexpr int GPS_TX_PIN =  17; // Conectar al pin RX del módulo NEO-7M
    // TODO: CAMBIAR LOS PINES POR LOS REALES
    constexpr uint8_t PIRO_PRINCIPAL_PIN                = 13;
    constexpr uint8_t CONTINUIDAD_PIRO_PRINCIPAL_PIN    = 39;

    constexpr uint8_t PIRO_DROGUE_PIN                   = 12;
    constexpr uint8_t CONTINUIDAD_PIRO_DROGUE_PIN       = 39;
    
    constexpr uint8_t FLASH_CS_PIN                      = 4;


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
    constexpr float VARIANZA_INICIAL_GIROSCOPIO     =   0.001f;
    constexpr float VARIANZA_INICIAL_ACELEROMETRO   =   0.01f;

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
    constexpr float PERIOD_SAMPLING_SENSORS_MS = 10.0f; // Muestreo cada 10 ms
    constexpr float FREC_SAMPLING_SENSORS_HZ = 1.0f / (PERIOD_SAMPLING_SENSORS_MS * 1e-3f);
}

enum comandosGlobale {
    CMD_ON_PIRO,
    CMD_OFF_PIRO,
    CMD_DISPARAR_PIRO,
    CMD_DEPLEGAR_DROGE
};

#endif //ACEMA_CTLR_CONFIG_H
/**
 *  Nota de cambios: 
 * - Se modigicó Sensors.cpp
 *  - Se cambió Actuators.cpp*/