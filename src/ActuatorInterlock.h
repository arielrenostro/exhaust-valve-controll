#ifndef ACTUATOR_INTERLOCK_H
#define ACTUATOR_INTERLOCK_H

#include <Arduino.h>

// Enforces the D7/D9 mutual-exclusion safety rule required because the motor
// winding and the position-sensor wire share a single physical conductor
// (see specs/exhaust-valve/actuator-interlock). This is the ONLY module
// allowed to touch D5, D6, D7, D9, or A0 - every other module must go
// through it to drive the motor or read the position.
namespace ActuatorInterlock
{
    enum class Direction : int8_t
    {
        Closing = -1,
        Stopped = 0,
        Opening = 1
    };

    void begin();

    // Drives the motor in `direction` at `dutyPercent` (0-100). Structurally
    // guarantees D9 is off (and settled) before D7 goes high, and applies a
    // direction-reversal dead-time if `direction` flips sign from the
    // previous call. Safe to call every control tick.
    void drive(Direction direction, uint8_t dutyPercent);

    // Stops the motor (EN low, both PWM outputs zero) without touching D9.
    void stopMotor();

    // Always disables the motor and waits the BTS7960 turn-off settle time
    // (regardless of whether it was already off), then takes one A0 sample
    // as fast as practical and disables the read MOSFET again. Does not
    // wait after disabling the read MOSFET - that wait happens inside the
    // next drive() call instead, whenever it comes. Returns the raw ADC
    // reading (0-1023). Blocking; callers must not log or do unrelated work
    // while this runs (see specs/exhaust-valve/diagnostics-logging).
    int readPositionRaw();
}

#endif
