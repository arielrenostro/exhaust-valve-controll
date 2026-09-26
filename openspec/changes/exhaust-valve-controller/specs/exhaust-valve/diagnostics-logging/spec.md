## Purpose

Gives visibility into firmware behavior over a 115200-baud serial connection for debugging, without the logging itself blocking or slowing down the time-critical interlock and control logic.

## ADDED Requirements

### Requirement: Serial initialization
The system SHALL open the serial port at 115200 baud during startup, before any other logging occurs.

#### Scenario: Startup
- **WHEN** the firmware boots
- **THEN** the serial port is initialized at 115200 baud before any other log line is written

### Requirement: Event logging
The system SHALL log discrete events — homing start and completion with recorded stop values and timings, control mode selection, faults, and fail-safe entry and exit — as they occur, without rate limiting.

#### Scenario: Homing completes
- **WHEN** the homing-calibration capability finishes a sweep
- **THEN** the system logs the recorded stop values, the recorded timings, and the selected control mode as an event, regardless of how recently a previous event was logged

### Requirement: Throttled telemetry
The system SHALL log a periodic telemetry line (target, current position or estimate, drive state, sensed current, battery voltage) no more often than a configured minimum interval.

#### Scenario: Continuous operation
- **WHEN** the system remains in normal control for longer than the configured telemetry interval
- **THEN** it logs at most one telemetry line per interval, regardless of how many control cycles occurred within that interval

### Requirement: No logging inside the interlock read window
The system SHALL NOT perform serial writes during the interval between disabling the motor driver and completing the position read that the actuator-interlock capability defines.

#### Scenario: Position read in progress
- **WHEN** the system is between disabling the motor driver and re-enabling it after a position read
- **THEN** no serial output is produced until that read cycle completes
