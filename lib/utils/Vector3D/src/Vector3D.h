//
// Created by zhl on 6/10/26.
//

#ifndef ACEMA_CTLR_VECTOR3D_H
#define ACEMA_CTLR_VECTOR3D_H

#include <iostream>

// TODO: CHEQUEAR MANUALMENTE CADA METODO

// ========================================================
// DEFINICIÓN DE LA INTERFAZ
// ========================================================
template <typename T>
struct Vector3D {
    T x;
    T y;
    T z;

    // Declaración de operadores miembro
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator*(const T& scalar) const;
    T operator*(const Vector3D& other) const;

    Vector3D& operator+=(const Vector3D& other);
    Vector3D& operator-=(const Vector3D& other);
};

// Declaración del operador externo (escalar * vector)
template <typename T>
Vector3D<T> operator*(const T& scalar, const Vector3D<T>& vec);


// ========================================================
// IMPLEMENTACIÓN DE LOS MÉTODOS
// ========================================================

// Suma
template <typename T>
Vector3D<T> Vector3D<T>::operator+(const Vector3D& other) const {
    return {x + other.x, y + other.y, z + other.z};
}

// Resta
template <typename T>
Vector3D<T> Vector3D<T>::operator-(const Vector3D& other) const {
    return {x - other.x, y - other.y, z - other.z};
}

// Vector * Escalar
template <typename T>
Vector3D<T> Vector3D<T>::operator*(const T& scalar) const {
    return {x * scalar, y * scalar, z * scalar};
}

// Producto Punto (Vector * Vector)
template <typename T>
T Vector3D<T>::operator*(const Vector3D& other) const {
    return (x * other.x) + (y * other.y) + (z * other.z);
}

// Asignación con Suma
template <typename T>
Vector3D<T>& Vector3D<T>::operator+=(const Vector3D& other) {
    x += other.x; y += other.y; z += other.z;
    return *this;
}

// Asignación con Resta
template <typename T>
Vector3D<T>& Vector3D<T>::operator-=(const Vector3D& other) {
    x -= other.x; y -= other.y; z -= other.z;
    return *this;
}

// Escalar * Vector (Función externa)
template <typename T>
Vector3D<T> operator*(const T& scalar, const Vector3D<T>& vec) {
    return {scalar * vec.x, scalar * vec.y, scalar * vec.z};
}




#endif //ACEMA_CTLR_VECTOR3D_H
