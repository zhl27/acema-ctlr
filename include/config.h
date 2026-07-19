//
// Created by zhl on 6/6/26.
//

#ifndef ACEMA_CTLR_CONFIG_H
#define ACEMA_CTLR_CONFIG_H


/*************************/
/*         GPIOs         */
/*************************/

#define BUZZER_PIN 25
#define WIRE_SDA_0 21
#define WIRE_SCL_0 22
#define SERVO_PIN 27
#define GPS_RX_PIN 16 // Conectar al pin TX del módulo NEO-7M
#define GPS_TX_PIN 17 // Conectar al pin RX del módulo NEO-7M


/*************************/
/*    DIRECCIONES I2C    */
/*************************/

#define MPU_ADDR 0x69
#define BMP280_ADDR 0x77

/***********************/
/*         MdE         */
/***********************/

#define UMBRAL_LANZAMIENTO 100 // TODO: falta implementar valor real
#define AREA_REFERENCIA_COHETE 0.01 // m^2, TODO: falta implementar valor real

/***********************/
/*         UART        */
/***********************/

#define SERIAL_BAUDRATE 115200


/************************/
/*       FREERTOS       */
/************************/

#define configSUPPORT_STATIC_ALLOCATION 1


/**********************/
/*        MAIN        */
/**********************/

constexpr int BUFFER_SIZE_BYTES = 1024*5;

#endif //ACEMA_CTLR_CONFIG_H
