//
// Created by zhl on 6/6/26.
//

#ifndef ACEMA_CTLR_CONFIG_H
#define ACEMA_CTLR_CONFIG_H


/*************************/
/*         GPIOs         */
/*************************/

#define BUZZER_PIN 25
#define WIRE_SDA 21
#define WIRE_SCL 22


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
