#include "DutyMapping.h"
#include "config.h"

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

float dutyPercentToTargetFraction(float dutyPercent)
{
    float fraction = clamp01(dutyPercent / 100.0f);
    if (cfg::INVERT_TARGET_DUTY)
    {
        fraction = 1.0f - fraction;
    }
    return fraction;
}
