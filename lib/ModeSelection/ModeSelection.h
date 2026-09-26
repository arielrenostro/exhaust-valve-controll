#ifndef MODE_SELECTION_H
#define MODE_SELECTION_H

enum class ControlMode
{
    Position,
    Time
};

// Selects POSITION control when the recorded stop readings differ by at
// least `minRangeCounts`, TIME control otherwise.
// See specs/exhaust-valve/homing-calibration - Control mode selection.
ControlMode selectControlMode(int closedStopReading, int openStopReading, int minRangeCounts);

#endif
