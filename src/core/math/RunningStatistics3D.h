/**
 * @file RunningStatistics3D.h
 * @brief Estadísticas incrementales para vectores tridimensionales.
 *
 * Encapsula tres instancias de RunningStatistics permitiendo calcular
 * estadísticas de vectores sin almacenar todas las muestras.
 *
 * Cada eje es tratado de forma independiente.
 */

#ifndef RUNNING_STATISTICS_3D_H
#define RUNNING_STATISTICS_3D_H

#include "Vector3f.h"
#include "RunningStatistics.h"
#include "Statistics.h"

namespace math 
{

class RunningStatistics3D
{
public:
    RunningStatistics3D() = default;

    /**
     * @brief Reinicia todas las estadísticas.
     */
    inline void reset() noexcept
    {
        // CORRECCIÓN: Cambiado de .clear() a .reset()
        m_x.reset();
        m_y.reset();
        m_z.reset();
    }

    /**
     * @brief Agrega una muestra.
     */
    inline void push(const Vector3f& sample) noexcept
    {
        m_x.push(sample.x);
        m_y.push(sample.y);
        m_z.push(sample.z);
    }

    /**
     * @brief Fusiona otro acumulador.
     */
    inline void merge(const RunningStatistics3D& other) noexcept
    {
        m_x.merge(other.m_x);
        m_y.merge(other.m_y);
        m_z.merge(other.m_z);
    }

    [[nodiscard]] inline uint32_t samples() const noexcept { return m_x.samples(); }
    [[nodiscard]] inline bool empty() const noexcept { return m_x.empty(); }

    //------------------------------------------------
    // Resultados rápidos
    //------------------------------------------------

    [[nodiscard]] inline Vector3f mean() const noexcept { return {m_x.mean(), m_y.mean(), m_z.mean()}; }
    [[nodiscard]] inline Vector3f variance() const noexcept { return {m_x.variance(), m_y.variance(), m_z.variance()}; }
    [[nodiscard]] inline Vector3f stddev() const noexcept { return {m_x.stddev(), m_y.stddev(), m_z.stddev()}; }
    [[nodiscard]] inline Vector3f rms() const noexcept { return {m_x.rms(), m_y.rms(), m_z.rms()}; }
    [[nodiscard]] inline Vector3f minimum() const noexcept { return {m_x.minimum(), m_y.minimum(), m_z.minimum()}; }
    [[nodiscard]] inline Vector3f maximum() const noexcept { return {m_x.maximum(), m_y.maximum(), m_z.maximum()}; }
    [[nodiscard]] inline Vector3f sem() const noexcept { return {m_x.sem(), m_y.sem(), m_z.sem()}; }

    //------------------------------------------------
    // Resultado completo
    //------------------------------------------------

    [[nodiscard]] inline Statistics3D statistics() const noexcept
    {
        Statistics3D stats;
        stats.x = m_x.statistics();
        stats.y = m_y.statistics();
        stats.z = m_z.statistics();
        return stats;
    }

private:
    RunningStatistics m_x;
    RunningStatistics m_y;
    RunningStatistics m_z;
};

} // namespace math

#endif // RUNNING_STATISTICS_3D_H
