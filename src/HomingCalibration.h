#ifndef HOMING_CALIBRATION_H
#define HOMING_CALIBRATION_H

#include <Arduino.h>
#include <ModeSelection.h>

// Startup stop-to-stop calibration sweep (specs/exhaust-valve/homing-calibration).
// Blocking by design: homing is a distinct startup phase, not part of the
// non-blocking RUNNING control loop.
namespace HomingCalibration
{
    struct Result
    {
        int closedStopRaw;
        int openStopRaw;
        unsigned long openTimeMs;
        unsigned long closeTimeMs;
        ControlMode mode;
    };

    // Drives to the closed stop, then to the open stop (fixed convention -
    // see design.md), recording each stop's position reading and the
    // elapsed time for each leg, then selects the control mode. Logs an
    // event with the recorded values on completion.
    Result run();
}

#endif
