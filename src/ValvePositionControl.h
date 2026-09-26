#ifndef VALVE_POSITION_CONTROL_H
#define VALVE_POSITION_CONTROL_H

#include <Arduino.h>
#include "HomingCalibration.h"

// Main control loop (specs/exhaust-valve/valve-position-control): drives the
// valve toward the ECU-requested target using the calibration produced by
// HomingCalibration, applies a one-shot confirmation push against either
// mechanical stop the moment it's reached (the valve has no return spring,
// but its drive mechanism is non-back-drivable, so no continuous holding
// force is needed), fails safe to fully open on D2 signal loss, and signals
// a mid-travel overcurrent fault so the caller can re-home.
namespace ValvePositionControl
{
    // (Re)initializes internal state from a fresh homing result. Call once
    // after every HomingCalibration::run(), including re-homing after a
    // fault.
    void reset(const HomingCalibration::Result &calibration);

    // Advances the control state machine. Call every loop() iteration - it
    // is non-blocking except for the bounded ActuatorInterlock drive/read
    // calls it makes internally at most once per cfg::T_DRIVE_MS tick.
    // Returns true exactly on the call where a mid-travel overcurrent fault
    // was raised; the caller must then re-run homing and call reset() again
    // before continuing to call update().
    bool update();
}

#endif
