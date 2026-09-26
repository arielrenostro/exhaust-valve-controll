#ifndef PWM_TARGET_READER_H
#define PWM_TARGET_READER_H

#include <Arduino.h>

// Non-blocking capture of a single PWM signal's duty cycle (e.g. the ECU's
// 100Hz exhaust valve target on D2), adapted from the interrupt + micros()
// technique in the project's original multi-pin src/pwm_reader.cpp, trimmed
// to one pin, with a Timer1-driven watchdog for signal-loss detection.
// See specs/exhaust-valve/pwm-target-input.
namespace PwmTargetReader
{
    // Starts capturing PWM edges on `pin` (must support attachInterrupt) and
    // starts the Timer1 watchdog that marks the reading invalid after
    // `timeoutMs` without an edge. Only one pin/instance is supported.
    void begin(uint8_t pin, unsigned long timeoutMs);

    // Duty cycle of the most recently completed PWM period, as a percentage
    // in [0, 100]. Stale/meaningless when isValid() is false.
    float getDutyPercent();

    // False once no edge has been observed on the pin for at least the
    // configured timeout (ECU signal lost, wiring fault, engine off, etc.).
    bool isValid();
}

#endif
