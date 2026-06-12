//
// Created by zhl on 6/6/26.
//

#ifndef ACEMA_CTLR_CONFIG_H
#define ACEMA_CTLR_CONFIG_H


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
