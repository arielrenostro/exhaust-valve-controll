#ifndef POSITION_DEADBAND_H
#define POSITION_DEADBAND_H

// True when the difference between the current position (or estimate) and
// the target exceeds the deadband, i.e. the motor should engage this tick.
// See specs/exhaust-valve/valve-position-control - Position deadband.
bool shouldDrive(float currentFraction, float targetFraction, float deadbandFraction);

#endif
