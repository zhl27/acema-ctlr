/**
 * @file Statistics.h
 * @brief Tipos de datos estadísticos utilizados por la librería.
 */

#ifndef STATISTICS_H
#define STATISTICS_H

#include <stdint.h>
#include "Vector3f.h" // Necesario para Statistics3D

namespace math 
{

struct Statistics
{
    uint32_t samples = 0;
    float mean = 0.0f;
    float variance = 0.0f;
    float stddev = 0.0f;
    float rms = 0.0f;
    float sem = 0.0f;
    float minimum = 0.0f;
    float maximum = 0.0f;

    [[nodiscard]] constexpr float range() const noexcept { return maximum - minimum; }
    [[nodiscard]] constexpr bool valid() const noexcept { return samples > 0; }
    [[nodiscard]] constexpr float sigma() const noexcept { return stddev; }
    
    [[nodiscard]] constexpr float cv() const noexcept 
    { 
        return mean != 0.0f ? stddev / mean : 0.0f; 
    }
    
    [[nodiscard]] constexpr float peakToPeak() const noexcept { return maximum - minimum; }
    [[nodiscard]] constexpr float bias() const noexcept { return mean; }
};

struct Statistics3D
{
    Statistics x;
    Statistics y;
    Statistics z;

    [[nodiscard]] constexpr Vector3f mean() const noexcept { return {x.mean, y.mean, z.mean}; }
    [[nodiscard]] constexpr Vector3f variance() const noexcept { return {x.variance, y.variance, z.variance}; }
    [[nodiscard]] constexpr Vector3f stddev() const noexcept { return {x.stddev, y.stddev, z.stddev}; }
    [[nodiscard]] constexpr Vector3f rms() const noexcept { return {x.rms, y.rms, z.rms}; }
    [[nodiscard]] constexpr Vector3f minimum() const noexcept { return {x.minimum, y.minimum, z.minimum}; }
    [[nodiscard]] constexpr Vector3f maximum() const noexcept { return {x.maximum, y.maximum, z.maximum}; }
    [[nodiscard]] constexpr Vector3f sem() const noexcept { return {x.sem, y.sem, z.sem}; }
};

} // namespace math

#endif // STATISTICS_H
