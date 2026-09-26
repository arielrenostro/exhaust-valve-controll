## Purpose

Determines the valve's physical travel limits and characteristic open/close timing at startup, since these vary per unit and must be known before any position- or time-based control can run.

## ADDED Requirements

### Requirement: Startup stop-to-stop sweep
On every power-on, before responding to the D2 target, the system SHALL drive the valve to one mechanical stop and then to the other.

#### Scenario: Normal boot
- **WHEN** the firmware starts
- **THEN** it performs a full sweep between the two mechanical stops before entering normal control

### Requirement: Pre-sweep release kick
Before starting the close-then-open sweep, the system SHALL unconditionally drive the valve open at a fixed high duty for a fixed short duration, without stall-current detection, regardless of where the valve currently rests.

#### Scenario: Valve already at the closed stop
- **WHEN** the homing sweep begins and the valve is already resting against the closed mechanical stop
- **THEN** the system performs the fixed-duration opening kick before driving toward the closed stop, so the closing-direction travel-timing capture does not record a near-zero elapsed time

#### Scenario: Valve elsewhere in its travel
- **WHEN** the homing sweep begins and the valve is not at the closed stop
- **THEN** the system still performs the same fixed-duration opening kick unconditionally, before proceeding with the close-then-open sweep

### Requirement: Current-based stop detection
The system SHALL detect that the valve has reached a mechanical stop by monitoring the BTS7960 current-sense reading for the currently active drive direction, without requiring a position sensor reading to do so. Detection SHALL be dynamic, comparing each reading against a baseline established early in that same drive rather than against a single fixed configured current value, so the trigger level reflects that direction's own current on this unit rather than a hand-calibrated number. Independently of the dynamic baseline, the system SHALL also stop immediately if the reading reaches a configured absolute current ceiling, as a safety backstop.

#### Scenario: Reaching a stop while driving
- **WHEN** the current-sense reading for the active drive direction rises above its baseline by a sustained, discrepant margin while driving toward a stop
- **THEN** the system stops driving in that direction and treats the valve as having reached that mechanical stop

#### Scenario: Brief current glitch, not a stop
- **WHEN** the current-sense reading rises briefly above the baseline but does not stay elevated for the sustained duration required to confirm a stop
- **THEN** the system continues driving instead of treating the glitch as the mechanical stop

#### Scenario: Absolute current ceiling reached
- **WHEN** the current-sense reading for the active drive direction reaches the configured absolute current ceiling, regardless of the dynamic baseline's state
- **THEN** the system immediately stops driving in that direction and treats the valve as having reached that mechanical stop

### Requirement: Position range capture
Immediately after detecting each stop, the system SHALL capture the position sensor's reading at that stop, using the actuator-interlock read protocol, and record it in RAM.

#### Scenario: Both stops captured
- **WHEN** both mechanical stops have been detected during the homing sweep
- **THEN** the system holds in RAM the position sensor reading captured at the fully closed stop and the reading captured at the fully open stop

### Requirement: Travel timing capture
The system SHALL record, as two independent values, the elapsed time taken to travel from one stop to the other in the opening direction and in the closing direction.

#### Scenario: Timing recorded
- **WHEN** the homing sweep completes
- **THEN** the system holds in RAM the elapsed opening time and the elapsed closing time as separate values

### Requirement: Control mode selection
The system SHALL select position-based control when the absolute difference between the recorded closed-stop and open-stop sensor readings is at least the configured minimum range, and SHALL select time-based control otherwise.

#### Scenario: Sensor shows meaningful range
- **WHEN** the difference between the recorded stop readings is at least the configured minimum range
- **THEN** the system enters normal control in position-based mode

#### Scenario: Sensor shows flat or unreliable range
- **WHEN** the difference between the recorded stop readings is below the configured minimum range
- **THEN** the system enters normal control in time-based mode, using the recorded opening and closing times to estimate position

### Requirement: Re-homing after mid-travel fault
The system SHALL perform a full homing sweep, discarding any previous calibration, whenever a mid-travel overcurrent fault is raised during normal control.

#### Scenario: Fault triggers re-homing
- **WHEN** a mid-travel overcurrent fault is raised during normal control
- **THEN** the system performs a new homing sweep, replacing the previously recorded stop positions and timings, before resuming normal control
