## Purpose

Prevents the motor driver and the position-sensor read circuit from ever being energized at the same time, since they share a single physical wire, protecting the microcontroller and the sensor from damage.

## ADDED Requirements

### Requirement: Mutual exclusion between drive and read
The system SHALL NOT enable the motor driver enable pin (D7) and the position-sensor read MOSFET (D9) at the same time, under any circumstance including startup, fault handling, and fail-safe operation.

#### Scenario: Requesting a position read while driving
- **WHEN** the control logic requests a position reading while the motor driver is enabled
- **THEN** the system disables the motor driver first, waits the configured settle time, and only then enables the read circuit

#### Scenario: Requesting to drive while reading
- **WHEN** the control logic requests to drive the motor while the position-read circuit is enabled
- **THEN** the system disables the read circuit first, waits the configured settle time, and only then enables the motor driver

### Requirement: Settle time enforcement
The system SHALL wait at least a configured settle time after disabling one side of the interlock before enabling the other side, to allow the shared wire's electrical state to stabilize.

#### Scenario: Switching from drive to read
- **WHEN** the motor driver is disabled in order to start a position read
- **THEN** the system waits at least the configured settle time before enabling the read circuit

#### Scenario: Switching from read to drive
- **WHEN** the read circuit is disabled in order to resume driving
- **THEN** the system waits at least the configured settle time before enabling the motor driver

### Requirement: Fast, bounded read window
Once the read circuit is enabled, the system SHALL take the position reading and disable the read circuit again as quickly as practical, minimizing the time the motor is prevented from driving.

#### Scenario: Position read cycle
- **WHEN** the read circuit becomes enabled
- **THEN** the system samples the position sensor and disables the read circuit again without performing unrelated work (such as serial logging) inside that window

### Requirement: Direction reversal dead-time
The system SHALL apply a dead-time with both motor drive outputs (R_PWM and L_PWM) held at zero whenever it reverses the commanded drive direction, before applying power in the new direction.

#### Scenario: Reversing direction
- **WHEN** the control logic switches the commanded drive direction from opening to closing, or from closing to opening
- **THEN** the system sets both R_PWM and L_PWM to zero, waits the configured dead-time, and only then applies power in the new direction
