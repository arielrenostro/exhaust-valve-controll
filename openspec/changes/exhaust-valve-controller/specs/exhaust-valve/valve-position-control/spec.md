## Purpose

Drives the valve toward the ECU-requested opening target using the calibration produced by homing, applies a one-shot confirmation push against either mechanical stop the moment it's reached since the valve has no return spring, and fails safe to fully open when the target signal is lost.

## ADDED Requirements

### Requirement: Closed-loop position control
When homing-calibration selects position-based mode, the system SHALL use a PID controller comparing the current sensor-derived position against the requested target to determine motor drive direction and magnitude.

#### Scenario: Target differs from current position
- **WHEN** the requested target differs from the current sensor-derived position by more than the configured position deadband
- **THEN** the system drives the motor toward the target, with drive magnitude determined by the PID controller

### Requirement: Open-loop time-based control
When homing-calibration selects time-based mode, the system SHALL estimate the current position by accumulating elapsed drive time scaled by the recorded opening or closing speed matching the active drive direction, and SHALL use this estimate in place of a sensor reading.

#### Scenario: Estimating during a direction change
- **WHEN** the commanded direction changes mid-travel in time-based mode
- **THEN** the system continues accumulating the position estimate using whichever recorded speed (opening or closing) matches the newly commanded direction

### Requirement: Seating push at mechanical stops
Because the valve has no return spring at either end of travel, the system SHALL, the moment the requested target is fully open or fully closed and the valve is confirmed at that stop for the first time since last being commanded away from it, drive against that stop at a configured duty for a configured duration, then release the motor. The system SHALL NOT apply this push again while the valve remains confirmed at that stop and the target remains unchanged.

#### Scenario: Reaching the open stop
- **WHEN** the requested target is fully open and the valve becomes confirmed at the open stop
- **THEN** the system drives toward open at the configured seating duty for the configured seating duration, then releases the motor

#### Scenario: Reaching the closed stop
- **WHEN** the requested target is fully closed and the valve becomes confirmed at the closed stop
- **THEN** the system drives toward closed at the configured seating duty for the configured seating duration, then releases the motor

#### Scenario: Already seated
- **WHEN** the valve remains confirmed at a stop it has already been pushed against, and the requested target has not changed away from that stop in the meantime
- **THEN** the system does not drive the motor

### Requirement: Mid-travel overcurrent fault
During normal, non-homing operation, if the active drive direction's current-sense reading exceeds the configured stall-current threshold while the valve is not confirmed at either recorded mechanical stop, the system SHALL immediately stop driving the motor and raise a mid-travel overcurrent fault.

#### Scenario: Obstruction detected
- **WHEN** the active direction's current exceeds the stall-current threshold while the current position or estimate is outside the deadband of both recorded stops
- **THEN** the system stops the motor immediately and raises a mid-travel overcurrent fault

### Requirement: Fail-safe on signal loss
When the pwm-target-input capability reports the D2 signal as invalid, the system SHALL set the requested target to fully open and SHALL apply the seating-push-at-mechanical-stops behavior toward that target for as long as the signal remains invalid.

#### Scenario: ECU signal lost during operation
- **WHEN** the D2 signal becomes invalid while the valve is at any position
- **THEN** the system drives the valve to the fully open stop, applies the seating push there, and remains released until a valid D2 signal is observed again

#### Scenario: Signal restored
- **WHEN** a valid D2 signal resumes after a fail-safe episode
- **THEN** the system resumes tracking the ECU-requested target using the previously selected control mode

### Requirement: Position deadband
The system SHALL NOT change the motor drive state for target deviations smaller than the configured position deadband, to avoid continuous on/off cycling caused by the drive/read interlock.

#### Scenario: Small deviation
- **WHEN** the difference between the current position (or estimate) and the requested target is within the configured deadband
- **THEN** the system does not drive the motor for that control cycle
