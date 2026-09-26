#ifndef TARGET_SOURCE_H
#define TARGET_SOURCE_H

#include <Arduino.h>

// Single switch point between the real ECU PWM signal (D2) and a bench
// potentiometer on A4, controlled by cfg::SIMULATE_TARGET_WITH_POTENTIOMETER.
// Exists so the rest of the firmware (ValvePositionControl) doesn't need to
// know or care which source is active - see
// specs/exhaust-valve/pwm-target-input.
namespace TargetSource
{
    void begin();

    // Requested duty cycle in [0,100], from whichever source is active.
    float getDutyPercent();

    // Signal-loss detection only applies to the real PWM source; the
    // potentiometer simulation is always considered valid (there's no
    // equivalent "signal lost" condition for a pot).
    bool isValid();
}

#endif
