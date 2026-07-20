/**
 * @file RunningStatistics.h
 * @brief Cálculo incremental de estadísticas utilizando el algoritmo de Welford.
 */

#ifndef RUNNINGSTATISTICS_H
#define RUNNINGSTATISTICS_H

#include <stdint.h>
#include "Statistics.h"

namespace math 
{

class RunningStatistics
{
public:
    RunningStatistics();

    void push(float sample);
    void reset() noexcept;
    void merge(const RunningStatistics& other) noexcept;

    [[nodiscard]] uint32_t samples() const noexcept;
    [[nodiscard]] float mean() const noexcept;
    [[nodiscard]] float variance() const noexcept;
    [[nodiscard]] float stddev() const noexcept;
    [[nodiscard]] float rms() const noexcept;
    [[nodiscard]] float sem() const noexcept;
    [[nodiscard]] float minimum() const noexcept;
    [[nodiscard]] float maximum() const noexcept;

    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] bool valid() const noexcept { return m_samples > 1; }
    
    [[nodiscard]] Statistics statistics() const noexcept;

private:
    uint32_t m_samples;
    double m_mean;
    double m_M2;
    double m_sumSquares;
    float m_minimum;
    float m_maximum;
};

} // namespace math

#endif // RUNNINGSTATISTICS_H
