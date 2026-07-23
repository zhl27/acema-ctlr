/**
 * @file Vector3f.h
 * @brief Tipo vectorial tridimensional ligero para sistemas embebidos.
 * @date 20-jul-2026
 * Esta implementación está diseñada para firmware de microcontroladores
 * y aplicaciones de navegación, control, sensores y filtros.
 *
 * Características:
 * - Tipo POD de 12 bytes (3 x float).
 * - Sin memoria dinámica.
 * - Sin STL.
 * - Header-only.
 * - Compatible con C++17.
 * - Operaciones matemáticas inline/constexpr.
 * - Adecuado para DMA, UART, CAN, telemetría y almacenamiento binario.
 */

#ifndef VECTOR3F_H
#define VECTOR3F_H

#include <cmath>
#include <cstdint>
namespace math
{

/**
 * @brief Tolerancia numérica por defecto para comparaciones.
 */
constexpr float kEpsilon = 1e-6f;


/**
 * @brief Vector3f como POD Puro (Standard-Layout y Trivial)
 * Sin constructores de usuario, sin inicializadores in-class.
 */
struct Vector3f
{
    float x;
    float y;
    float z;
};
/*
struct Vector3f
{
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    // CONSTRUCTORES EXPLÍCITOS CORREGIDOS:
    // Constructor por defecto obligatorio
    constexpr Vector3f() noexcept = default;

    // Constructor de inicialización que resuelve el error de las llaves {}
    constexpr Vector3f(float x_, float y_, float z_) noexcept 
        : x(x_), y(y_), z(z_) {}
};
*/
// -----------------------------------------------------------------------------
// Constantes
// -----------------------------------------------------------------------------

[[nodiscard]] constexpr inline Vector3f zero()
{
    // static_assert(std::is_trivial<math::Vector3f>::value, "Vector3f debe ser Trivial");
    //static_assert(std::is_standard_layout<math::Vector3f>::value, "Vector3f debe ser Standard-Layout");

    return Vector3f{0.0f, 0.0f, 0.0f};
}

[[nodiscard]] constexpr inline Vector3f unitX()
{
    return Vector3f{1.0f, 0.0f, 0.0f};
}

[[nodiscard]] constexpr inline Vector3f unitY()
{
    return Vector3f{0.0f, 1.0f, 0.0f};
}

[[nodiscard]] constexpr inline Vector3f unitZ()
{
    return Vector3f{0.0f, 0.0f, 1.0f};
}

// -----------------------------------------------------------------------------
// Operadores básicos
// -----------------------------------------------------------------------------

[[nodiscard]] constexpr inline Vector3f operator+(
        const Vector3f& a,
        const Vector3f& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

[[nodiscard]] constexpr inline Vector3f operator-(
        const Vector3f& a,
        const Vector3f& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

[[nodiscard]] constexpr inline Vector3f operator-(
        const Vector3f& v)
{
    return {-v.x, -v.y, -v.z};
}

[[nodiscard]] constexpr inline Vector3f operator*(
        const Vector3f& v,
        float scalar)
{
    return {v.x * scalar, v.y * scalar, v.z * scalar};
}

[[nodiscard]] constexpr inline Vector3f operator*(
        float scalar,
        const Vector3f& v)
{
    return v * scalar;
}

[[nodiscard]] inline Vector3f operator/(
        const Vector3f& v,
        float scalar)
{
    if(std::fabs(scalar) <= kEpsilon)
        return zero();

    return {v.x / scalar, v.y / scalar, v.z / scalar};
}

inline Vector3f& operator+=(
        Vector3f& a,
        const Vector3f& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Vector3f& operator-=(
        Vector3f& a,
        const Vector3f& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline Vector3f& operator*=(
        Vector3f& a,
        float scalar)
{
    a.x *= scalar;
    a.y *= scalar;
    a.z *= scalar;
    return a;
}

inline Vector3f& operator/=(
        Vector3f& a,
        float scalar)
{
    if(std::fabs(scalar) > kEpsilon)
    {
        a.x /= scalar;
        a.y /= scalar;
        a.z /= scalar;
    }
    else
    {
        a = zero();
    }
    return a;
}

// -----------------------------------------------------------------------------
// Comparaciones
// -----------------------------------------------------------------------------

[[nodiscard]] constexpr inline bool operator==(
        const Vector3f& a,
        const Vector3f& b)
{
    return (a.x == b.x) &&
           (a.y == b.y) &&
           (a.z == b.z);
}

[[nodiscard]] constexpr inline bool operator!=(
        const Vector3f& a,
        const Vector3f& b)
{
    return !(a == b);
}

/**
 * @brief Comparación aproximada.
 */
[[nodiscard]] inline bool almostEqual(
        const Vector3f& a,
        const Vector3f& b,
        float epsilon = kEpsilon)
{
    return (std::fabs(a.x - b.x) <= epsilon) &&
           (std::fabs(a.y - b.y) <= epsilon) &&
           (std::fabs(a.z - b.z) <= epsilon);
}

// -----------------------------------------------------------------------------
// Operaciones matemáticas
// -----------------------------------------------------------------------------

/**
 * @brief Producto escalar.
 */
[[nodiscard]] constexpr inline float dot(
        const Vector3f& a,
        const Vector3f& b)
{
    return (a.x * b.x) +
           (a.y * b.y) +
           (a.z * b.z);
}

/**
 * @brief Producto vectorial.
 */
[[nodiscard]] constexpr inline Vector3f cross(
        const Vector3f& a,
        const Vector3f& b)
{
    return
    {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    };
}

/**
 * @brief Norma al cuadrado.
 */
[[nodiscard]] constexpr inline float squaredNorm(
        const Vector3f& v)
{
    return dot(v, v);
}

/**
 * @brief Norma euclídea.
 */
[[nodiscard]] inline float norm(
        const Vector3f& v)
{
    return std::sqrt(squaredNorm(v));
}

/**
 * @brief Distancia euclídea entre dos vectores.
 */
[[nodiscard]] inline float distance(
        const Vector3f& a,
        const Vector3f& b)
{
    return norm(a - b);
}

/**
 * @brief Verifica si el vector es aproximadamente nulo.
 */
[[nodiscard]] inline bool isZero(
        const Vector3f& v,
        float epsilon = kEpsilon)
{
    return squaredNorm(v) <= (epsilon * epsilon);
}

/**
 * @brief Devuelve una copia normalizada.
 */
[[nodiscard]] inline Vector3f normalized(
        const Vector3f& v)
{
    const float n = norm(v);

    if(n <= kEpsilon)
        return zero();

    return v / n;
}

/**
 * @brief Normaliza el vector in-place.
 */
inline void normalize(
        Vector3f& v)
{
    v = normalized(v);
}

/**
 * @brief Proyección de a sobre b.
 */
[[nodiscard]] inline Vector3f project(
        const Vector3f& a,
        const Vector3f& b)
{
    const float denom = squaredNorm(b);

    if(denom <= kEpsilon)
        return zero();

    return b * (dot(a, b) / denom);
}

/**
 * @brief Interpolación lineal entre dos vectores.
 *
 * t = 0 devuelve a.
 * t = 1 devuelve b.
 */
[[nodiscard]] constexpr inline Vector3f lerp(
        const Vector3f& a,
        const Vector3f& b,
        float t)
{
    return
    {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

/**
 * @brief Ángulo entre dos vectores en radianes.
 */
[[nodiscard]] inline float angle(
        const Vector3f& a,
        const Vector3f& b)
{
    const float na = norm(a);
    const float nb = norm(b);

    if(na <= kEpsilon || nb <= kEpsilon)
        return 0.0f;

    float c = dot(a, b) / (na * nb);

    // Saturación por errores numéricos.
    if(c > 1.0f) c = 1.0f;
    if(c < -1.0f) c = -1.0f;

    return std::acos(c);
}

} // namespace math


#endif // VECTOR3F_H
