#include "RunningStatistics.h"
#include <cmath>
#include <cfloat>

namespace math 
{

RunningStatistics::RunningStatistics()
{
    reset();
}

void RunningStatistics::reset() noexcept
{
    m_samples = 0;
    m_mean = 0.0;
    m_M2 = 0.0;
    m_sumSquares = 0.0;
    m_minimum = FLT_MAX;
    m_maximum = -FLT_MAX;
}

bool RunningStatistics::empty() const noexcept
{
    return m_samples == 0;
}

void RunningStatistics::merge(const RunningStatistics& other) noexcept
{
    if (other.empty()) return;

    if (empty())
    {
        *this = other;
        return;
    }

    const uint32_t totalSamples = m_samples + other.m_samples;
    const double delta = other.m_mean - m_mean;

    m_mean = (m_mean * m_samples + other.m_mean * other.m_samples) / totalSamples;
    
    m_M2 += other.m_M2 + delta * delta * m_samples * other.m_samples / totalSamples;
    
    m_sumSquares += other.m_sumSquares;

    if (other.m_minimum < m_minimum) m_minimum = other.m_minimum;
    if (other.m_maximum > m_maximum) m_maximum = other.m_maximum;

    m_samples = totalSamples;
}

void RunningStatistics::push(float sample)
{
    if (sample < m_minimum) m_minimum = sample;
    if (sample > m_maximum) m_maximum = sample;

    ++m_samples;

    const double delta = sample - m_mean;
    m_mean += delta / m_samples;
    
    const double delta2 = sample - m_mean;
    m_M2 += delta * delta2;
    
    m_sumSquares += static_cast<double>(sample) * sample;
}

uint32_t RunningStatistics::samples() const noexcept { return m_samples; }

float RunningStatistics::mean() const noexcept { return static_cast<float>(m_mean); }

float RunningStatistics::variance() const noexcept
{
    if (m_samples < 2) return 0.0f;
    return static_cast<float>(m_M2 / (m_samples - 1));
}

float RunningStatistics::stddev() const noexcept
{
    return std::sqrt(variance());
}

float RunningStatistics::rms() const noexcept
{
    if (m_samples == 0) return 0.0f;
    return std::sqrt(static_cast<float>(m_sumSquares / m_samples));
}

float RunningStatistics::sem() const noexcept
{
    if (m_samples == 0) return 0.0f;
    return stddev() / std::sqrt(static_cast<float>(m_samples));
}

float RunningStatistics::minimum() const noexcept
{
    if (m_samples == 0) return 0.0f;
    return m_minimum;
}

float RunningStatistics::maximum() const noexcept
{
    if (m_samples == 0) return 0.0f;
    return m_maximum;
}

Statistics RunningStatistics::statistics() const noexcept
{
    Statistics s;
    s.samples = samples();
    s.mean = mean();
    s.variance = variance();
    s.stddev = stddev();
    s.rms = rms();
    s.sem = sem();
    s.minimum = minimum();
    s.maximum = maximum();
    return s;
}

} // namespace math
