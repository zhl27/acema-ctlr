//
// Created by zhl on 6/10/26.
//

#ifndef ACEMA_CTLR_FUNCIONES_DATOS_H
#define ACEMA_CTLR_FUNCIONES_DATOS_H

#include <cstdint>

#include "Vector3D.h"
#include "config.h"


Vector3D<int16_t> f_sensor_get_aceleracion();
Vector3D<int16_t> f_sensor_get_aceleracion_angular(); // aceleracion angular

int32_t f_sensor_get_presion(); // 20 bits

// Estos 2 de abajo se pueden reemplazar por los de cada sensor correspondiente.
int32_t f_sensor_get_temperatura_bmp(); // sacado de la bmp280 --> 20 bits
int16_t f_sensor_get_temperatura_mpu(); // sacado de la mpu6050

int8_t f_sensor_get_temperatura(); // TODO: bmp280 y mpu5060 ambos tienen temperatura, pero se miden de forma diferente, por lo que se necesitan dos funciones distintas para cada sensor. Podemos hacer un promedio de ambos.

// derivados de los datos de arriba --> obtenidos de la integracion
int32_t f_sensor_get_altitud(); // Creo que las MCU de los sensores los calcula, pero esos son lentos, preferimos usar esp32 para inferirlo.
Vector3D<int16_t> f_sensor_get_momentum(); // Es la inercia. Se puede calcular a partir de la aceleración y la masa del cohete, pero también se puede usar un modelo físico más complejo que tenga en cuenta la aerodinámica y la resistencia del aire.
Vector3D<int16_t> f_sensor_get_velocidad(); // Obtenida de la integracion de la aceleracion, pero con correcciones de drift y ruido. Se puede usar un filtro de Kalman o similar para mejorar la estimación.
Vector3D<int16_t> f_sensor_get_posicion(); // Nos permitiria dibujar un trazo del cohete en la GUI GSE

// datos utilizados por el mcp controlador
int16_t f_sensor_get_densidad_aire(); // rho
// vector3d<int16_t> f_sensor_get_velocidad();  // altura es lo mismo que posición vertical
// int32_t f_sensor_get_altitud(); // ya definido arriba
// objetivo h --> altura objetivo
Vector3D<int16_t> f_sensor_get_drag(); // Drag en Newtons = 1/2 * rho * AreaDeReferencia * Cd * V^2




#endif //ACEMA_CTLR_FUNCIONES_DATOS_H
