#include "Pid.h"

namespace
{
    float clamp(float value, float lo, float hi)
    {
        if (value < lo)
        {
            return lo;
        }
        if (value > hi)
        {
            return hi;
        }
        return value;
    }
}

PidController::PidController(float kp, float ki, float kd)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_integral(0.0f), m_previousError(0.0f), m_hasPreviousError(false)
{
}

void PidController::reset()
{
    m_integral = 0.0f;
    m_previousError = 0.0f;
    m_hasPreviousError = false;
}

float PidController::update(float error, float dtSeconds)
{
    m_integral += error * dtSeconds;

    float derivative = 0.0f;
    if (m_hasPreviousError && dtSeconds > 0.0f)
    {
        derivative = (error - m_previousError) / dtSeconds;
    }
    m_previousError = error;
    m_hasPreviousError = true;

    const float output = m_kp * error + m_ki * m_integral + m_kd * derivative;
    return clamp(output, -1.0f, 1.0f);
}
