## Why

The project currently has only a PlatformIO skeleton and bench-test sketches (`src/main.cpp`, `src/leitor_pwm.cpp`) used to characterize the ECU's PWM signal — there is no actual exhaust valve controller firmware. We need real firmware for an Arduino Nano that reads a 100Hz duty-cycle target from the ECU, drives a DC motor through a BTS7960 module to match that target on a potentiometer position sensor, and does so safely despite a hardware constraint: the position sensor and the motor share a wire, so reading the sensor and driving the motor can never happen at the same time.

## What Changes

- **BREAKING**: Removes the bench-test sketches (`src/main.cpp`, `src/leitor_pwm.cpp`) and the unused `arduino-pwm-frequency-library` dependency; the Nano's default PWM frequency is used for the motor outputs. `src/pwm_reader.cpp`'s interrupt+timeout technique is reused as the basis for the new non-blocking D2 reader.
- Adds a non-blocking reader for the ECU's 100Hz PWM target on D2 (interrupt-driven, with signal-loss/timeout detection).
- Adds a hardware safety interlock so D7 (motor enable) and D9 (position-sensor MOSFET) are never enabled together, with dead-time on every mode switch, since A0 and the motor winding share the same physical wire.
- Adds a startup homing routine that drives the valve from stop to stop, using BTS7960 current sensing (A1/A2) to detect each mechanical stop (batente), recording the sensor's min/max ADC values and the elapsed open/close times in RAM.
- Adds dual control-mode selection: closed-loop PID on the potentiometer reading when it shows a meaningful range across the homed travel (>= 100 ADC counts), otherwise open-loop time-based (dead-reckoning) control using the calibrated open/close speeds.
- Adds a "seating push" at each commanded extreme (fully open or fully closed): the moment the valve is first confirmed at that stop, it drives against it at a configured duty for a configured duration, then releases the motor — the valve has no return spring, but its drive mechanism is non-back-drivable, so no continuous holding force is needed once seated.
- Adds a fail-safe: if the D2 PWM signal is lost or invalid, the valve drives to fully open and applies the same seating push there, then releases, until a valid signal returns.
- Adds mid-travel overcurrent fault handling: an unexpected current spike outside of homing cuts the motor and triggers an automatic re-homing pass before control resumes.
- Adds non-blocking, throttled serial debug logging at 115200 baud, split into always-on rare event logs (homing, fault, mode changes) and a rate-limited telemetry heartbeat, both kept out of the time-critical read window.
- Centralizes all tunable calibration values (drive/settle timings, ADC prescaler, stall-current thresholds, PID gains, deadbands, seating push duty/duration, etc.) as named constants for later bench calibration.

## Capabilities

### New Capabilities
- `exhaust-valve/pwm-target-input`: Non-blocking capture of the ECU's 100Hz PWM duty on D2, including signal-loss/timeout detection.
- `exhaust-valve/actuator-interlock`: The D7/D9 mutual-exclusion safety rule for the shared motor/sensor wire, including switch dead-time and motor direction-reversal dead-time.
- `exhaust-valve/homing-calibration`: The startup stop-to-stop calibration routine (current-based stall detection, min/max position capture, open/close timing capture, control-mode selection).
- `exhaust-valve/valve-position-control`: The main control loop — PID or time-based positioning toward the D2-derived target, a seating push at the extremes, fail-safe-to-open on signal loss, and mid-travel overcurrent fault handling with auto re-homing.
- `exhaust-valve/diagnostics-logging`: Non-blocking serial debug logging (115200 baud) split into event and throttled telemetry tiers.

### Modified Capabilities
(none — greenfield project, no existing specs)

## Impact

- Replaces all firmware in `src/` (and the header-only `src/pwm_reader.cpp`, which moves to `lib/` in a simplified single-pin form) with the new controller firmware.
- Removes the `phatpaul/arduino-pwm-frequency-library` dependency from `platformio.ini`; keeps `paulstoffregen/TimerOne`.
- Introduces a centralized tunable-constants header (e.g. `include/config.h`) that the user will populate with bench-measured values before flashing to the vehicle.
- No changes to the fixed pinout (D2, D5, D6, D7, D9, A0-A3) already wired on the hardware.
