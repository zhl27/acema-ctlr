//
// Created by zhl on 6/10/26.
//

#include "funciones_datos.h"


/*
 * Estas funciones sirven para centralizar la obtención de datos a través de una forma funcional.
 *
 */


Vector3D<int16_t> f_sensor_get_aceleracion() {
    return Vector3D<int16_t>{0, 0, 0};
}
Vector3D<int16_t> f_sensor_get_aceleracion_angular() {
    return Vector3D<int16_t>{0, 0, 0};
}

int32_t f_sensor_get_presion() { // 20 bits
    return 0;
}

int32_t f_sensor_get_temperatura_bmp() {
    return 0;
}

int16_t f_sensor_get_temperatura_mpu() {
    return 0;
}

int8_t f_sensor_get_temperatura() {
    // bmp280 y mpu5060 ambos tienen temperatura, pero se miden de forma diferente, por lo que se necesitan dos funciones distintas para cada sensor. Hacemos un promedio de ambos.
    return ( f_sensor_get_temperatura_bmp() + f_sensor_get_temperatura_mpu()) / 2; // TODO: considerar aplicar un SimpleKalmanFilter para obtener una estimación más robusta de la temperatura combinando ambas fuentes de datos, en lugar de un simple promedio. Esto ayudaría a reducir el ruido y mejorar la precisión de la medición de temperatura.
}

int32_t f_sensor_get_altitud() {
    return 0;
}
Vector3D<int16_t> f_sensor_get_momentum() {
    return Vector3D<int16_t>{0, 0, 0};
}
Vector3D<int16_t> f_sensor_get_velocidad() {
    return Vector3D<int16_t>{0, 0, 0};
}
Vector3D<int16_t> f_sensor_get_posicion() {
    return Vector3D<int16_t>{0, 0, 0};
}

int16_t f_sensor_get_densidad_aire() { // rho --> se calcula con el barometro
    return 0;
}

Vector3D<int16_t> f_sensor_get_drag() {
    return Vector3D<int16_t>{0, 0, 0};
    //return 1/2 * f_sensor_get_densidad_aire() * AREA_REFERENCIA_COHETE * Cd * V*V;
}