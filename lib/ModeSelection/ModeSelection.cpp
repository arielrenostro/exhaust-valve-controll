#include "ModeSelection.h"

ControlMode selectControlMode(int closedStopReading, int openStopReading, int minRangeCounts)
{
    int range = openStopReading - closedStopReading;
    if (range < 0)
    {
        range = -range;
    }
    return range >= minRangeCounts ? ControlMode::Position : ControlMode::Time;
}
