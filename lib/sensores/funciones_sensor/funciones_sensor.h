//
// Created by zhl on 6/8/26.
//

#ifndef ACEMA_CTLR_FUNCIONES_SENSOR_H
#define ACEMA_CTLR_FUNCIONES_SENSOR_H
#include <cstdint>


template <typename T>
struct vector3d {
    T x;
    T y;
    T z;
};


vector3d<int16_t> f_sensor_get_aceleracion(uint8_t sensor_id);
vector3d<int16_t> f_sensor_get_aceleracion_angular(uint8_t sensor_id); // aceleracion angular

vector3d<int32_t> f_sensor_get_presion(uint8_t sensor_id); // 20 bits

// Estos 2 de abajo se pueden reemplazar por los de cada sensor correspondiente.
vector3d<int32_t> f_sensor_get_temperatura_bmp(uint8_t sensor_id); // sacado de la bmp280 --> 20 bits
vector3d<int16_t> f_sensor_get_temperatura_mpu(uint8_t sensor_id); // sacado de la mpu6050

vector3d<int8_t> f_sensor_get_temperatura(uint8_t sensor_id); // TODO: bmp280 y mpu5060 ambos tienen temperatura, pero se miden de forma diferente, por lo que se necesitan dos funciones distintas para cada sensor. Podemos hacer un promedio de ambos.

// derivados de los datos de arriba --> obtenidos de la integracion
int32_t f_sensor_get_altitud(uint8_t sensor_id); // Creo que las MCU de los sensores los calcula, pero esos son lentos, preferimos usar esp32 para inferirlo.
vector3d<int16_t> f_sensor_get_momentum(uint8_t sensor_id); // Es la inercia. Se puede calcular a partir de la aceleración y la masa del cohete, pero también se puede usar un modelo físico más complejo que tenga en cuenta la aerodinámica y la resistencia del aire.
vector3d<int16_t> f_sensor_get_velocidad(uint8_t sensor_id); // Obtenida de la integracion de la aceleracion, pero con correcciones de drift y ruido. Se puede usar un filtro de Kalman o similar para mejorar la estimación.
vector3d<int16_t> f_sensor_get_posicion(uint8_t sensor_id); // Nos permitiria dibujar un trazo del cohete en la GUI GSE




#endif //ACEMA_CTLR_FUNCIONES_SENSOR_H
