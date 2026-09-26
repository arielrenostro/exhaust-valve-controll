#include "AnalogTargetMapping.h"

float analogCountsToPercent(int rawCounts, int maxCounts)
{
    if (maxCounts <= 0)
    {
        return 0.0f;
    }

    float fraction = (float)rawCounts / (float)maxCounts;
    if (fraction < 0.0f)
    {
        fraction = 0.0f;
    }
    if (fraction > 1.0f)
    {
        fraction = 1.0f;
    }
    return fraction * 100.0f;
}
