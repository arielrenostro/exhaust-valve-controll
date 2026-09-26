#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Non-blocking-friendly serial diagnostics at 115200 baud
// (specs/exhaust-valve/diagnostics-logging), split into two tiers:
//   - event(): rare, always-on, never rate-limited (homing, faults, mode
//     changes, fail-safe entry/exit)
//   - telemetry(): throttled to at most one line per
//     cfg::TELEMETRY_LOG_INTERVAL_MS
// Neither function is ever called from inside ActuatorInterlock's read
// window - ActuatorInterlock itself contains no logging calls, and every
// caller of ActuatorInterlock::readPositionRaw() is blocked on that single
// call while it runs, so no log line can be interleaved with it.
namespace Logger
{
    void begin();

    void event(const String &message);

    void telemetry(float targetFraction, float positionFraction, int8_t activeDirection, float currentAmps, float batteryVolts);
}

#endif
