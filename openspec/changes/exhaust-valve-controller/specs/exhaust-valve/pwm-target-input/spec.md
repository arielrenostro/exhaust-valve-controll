## Purpose

Captures the exhaust valve opening target that the ECU broadcasts as a 100Hz PWM duty cycle on pin D2, without blocking the rest of the firmware, and detects when that signal is missing or invalid.

## ADDED Requirements

### Requirement: Non-blocking duty capture
The system SHALL measure the duty cycle of the PWM signal on pin D2 using interrupt-driven edge timing, and SHALL NOT use blocking calls such as `pulseIn` or `delay` to do so.

#### Scenario: Valid PWM present
- **WHEN** the ECU drives a PWM signal on D2 with a stable period near 10ms (100Hz)
- **THEN** the system computes and updates a duty-cycle percentage while other tasks (logging, motor control, etc.) continue running in the same loop iteration without being paused to wait for D2 edges

### Requirement: Signal loss detection
The system SHALL detect when the D2 PWM signal stops toggling or becomes implausible (stuck high or stuck low) within a bounded timeout, and SHALL report this as an invalid-target condition.

#### Scenario: ECU signal disappears
- **WHEN** no edge transition is observed on D2 for longer than the configured timeout
- **THEN** the system marks the target duty as invalid and exposes the signal-loss condition for the control loop to consume

#### Scenario: Signal recovers
- **WHEN** a valid, stable PWM signal resumes on D2 after a signal-loss condition
- **THEN** the system clears the invalid-target condition and resumes reporting a duty-cycle percentage

### Requirement: Duty-to-target mapping
The system SHALL map a 0% measured duty cycle to a fully closed valve target and a 100% measured duty cycle to a fully open valve target.

#### Scenario: Mid-range duty
- **WHEN** the measured duty cycle is X percent, with X between 0 and 100
- **THEN** the requested valve opening target is X percent of the full travel range
