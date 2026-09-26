#include "PositionDeadband.h"

bool shouldDrive(float currentFraction, float targetFraction, float deadbandFraction)
{
    float diff = targetFraction - currentFraction;
    if (diff < 0.0f)
    {
        diff = -diff;
    }
    return diff > deadbandFraction;
}
