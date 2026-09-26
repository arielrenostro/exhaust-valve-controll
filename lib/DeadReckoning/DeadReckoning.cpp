#include "DeadReckoning.h"

namespace
{
    float clamp01(float value)
    {
        if (value < 0.0f)
        {
            return 0.0f;
        }
        if (value > 1.0f)
        {
            return 1.0f;
        }
        return value;
    }
}

DeadReckoningEstimator::DeadReckoningEstimator(unsigned long openTimeMs, unsigned long closeTimeMs, float initialFraction)
    : m_openTimeMs(openTimeMs), m_closeTimeMs(closeTimeMs), m_fraction(clamp01(initialFraction))
{
}

void DeadReckoningEstimator::update(int direction, unsigned long elapsedMs)
{
    if (direction > 0 && m_openTimeMs > 0)
    {
        m_fraction = clamp01(m_fraction + (float)elapsedMs / (float)m_openTimeMs);
    }
    else if (direction < 0 && m_closeTimeMs > 0)
    {
        m_fraction = clamp01(m_fraction - (float)elapsedMs / (float)m_closeTimeMs);
    }
}

void DeadReckoningEstimator::resyncTo(float fraction)
{
    m_fraction = clamp01(fraction);
}

float DeadReckoningEstimator::fraction() const
{
    return m_fraction;
}
