## 1. Project cleanup and shared configuration

- [x] 1.1 Remove `src/main.cpp` and `src/leitor_pwm.cpp` bench-test sketches, remove `phatpaul/arduino-pwm-frequency-library` from `platformio.ini`, and add a minimal placeholder `src/main.cpp` (empty `setup()`/`loop()`); verify `pio run` builds successfully.
- [x] 1.2 Create `include/config.h` with all tunable constants grouped by concern (drive/settle timing, ADC prescaler, current thresholds in Amps plus the Amps-to-ADC conversion constant, PID gains, position deadband, holding duty/current, mode-selection minimum range, D2 signal timeout, telemetry interval), each with a placeholder value and a comment explaining what it affects; verify `pio run` still builds after including it from `main.cpp`.
- [x] 1.3 Port `src/pwm_reader.cpp`'s interrupt+watchdog technique into a new single-pin `lib/PwmTargetReader` module and delete the old multi-pin, header-only version from `src/`; verify `pio run` builds with the new lib linked.

## 2. PWM target input (D2)

- [x] 2.1 Implement non-blocking duty capture on D2 using a CHANGE interrupt and `micros()`, exposing a duty-cycle percentage getter; verify with a bench 100Hz PWM source that the logged duty tracks known input values while the rest of the loop keeps running.
- [x] 2.2 Implement a Timer1-based signal-loss watchdog (via the existing `TimerOne` dependency) that marks the target invalid after no edge for `D2_SIGNAL_TIMEOUT_MS`; verify by disconnecting the bench PWM source and confirming the invalid flag sets, then clears on reconnect, via serial log.
- [x] 2.3 Implement the duty-to-target mapping (0% = closed, 100% = open) as a pure function; verify with a native PlatformIO unit test covering 0%, 50%, 100%, and out-of-range clamping.

## 3. Actuator interlock

- [x] 3.1 Implement the D7/D9 mutual-exclusion state machine (DRIVE <-> READ) enforcing `T_SETTLE_MS` dead-time on every transition, structured so D7 and D9 can never both be HIGH; verify on the bench with a logic analyzer or multimeter across repeated drive/read transitions that D7 and D9 are never simultaneously HIGH.
- [x] 3.2 Implement the bounded position-read routine (enable D9, sample A0 at `ADC_PRESCALER`, disable D9) minimizing time spent in the read window; verify by timing the window with `micros()` and logging it, confirming it stays within the expected bound.
- [x] 3.3 Implement direction-reversal dead-time (zero both R_PWM/L_PWM, wait `T_SETTLE_MS`, then apply the new direction) whenever the commanded drive direction changes; verify on the bench with an oscilloscope or multimeter across repeated reversals that R_PWM and L_PWM are never both driving nonzero.

## 4. Sensing helpers

- [ ] 4.1 Implement current-sense reading and Amps conversion for A1/A2 using the 0.00V = 0A / 4.0V = 3.4A scale, with `readRAmps`/`readLAmps` block-averaging over a caller-supplied `windowMs` (see design.md Decision 17); verify with a native unit test checking known conversions (0V -> 0A, 4.0V -> 3.4A, a mid-scale value).
- [x] 4.2 Implement battery-voltage reading on A3 for telemetry; verify by logging a plausible voltage value on the bench.

## 5. Homing calibration

- [x] 5.1 Implement the stop-to-stop sweep, using `STALL_CURRENT_THRESHOLD_R_AMPS`/`STALL_CURRENT_THRESHOLD_L_AMPS` on the active direction's current sense to detect arrival at each mechanical stop; verify on the bench that driving into each end stops automatically at the expected current spike, confirmed via the serial event log.
- [x] 5.2 Implement position-range capture (sensor reading at each stop, via the interlock's read routine) and travel-timing capture (elapsed open time and close time); verify via serial event log showing the recorded min/max ADC values and both timings after a bench sweep.
- [x] 5.3 Implement control-mode selection (POSITION vs. TIME, using `POSITION_MIN_RANGE_COUNTS`) and expose the selected mode; verify with a native unit test for the selection threshold logic, plus a bench confirmation that the real sensor's ~443-count range selects POSITION mode.
- [x] 5.4 Wire homing to run once at boot before entering RUNNING, and to re-run (discarding prior calibration) whenever a mid-travel fault is raised; verify via serial event log showing homing at boot, and again after simulating a mid-travel fault (task 6.5).
- [x] 5.5 Add an unconditional, fixed-duration (`HOMING_PRE_OPEN_KICK_MS` = 100ms) full-duty (`HOMING_PRE_OPEN_KICK_DUTY_PERCENT` = 100%) opening kick before the close-then-open sweep, with no stall-current detection during the kick; verify on the bench that starting homing with the valve already resting at the closed stop no longer logs a near-zero closing-leg travel time.
- [x] 5.6 Replace `driveToStop()`'s fixed per-direction Amp threshold with dynamic, baseline-relative stall detection (a baseline learned once from a short window sized to fit inside the sub-second sweep, sustained-spike confirmation) operating on `CurrentSensing`'s already-filtered reading at `CURRENT_SENSE_FILTER_WINDOW_MS` (block-averaging itself no longer lives in `driveToStop()` - see design.md Decision 17), plus an absolute `HOMING_CURRENT_HARD_LIMIT_AMPS` safety ceiling read at the shorter `CURRENT_SENSE_HARD_LIMIT_WINDOW_MS`, using the `HOMING_CURRENT_*` constants in `include/config.h`; verify on the bench that the locked baseline reflects free-running current (not already-stalled current), the sweep stops promptly at each genuine mechanical stop, and it no longer times out at `HOMING_MAX_DRIVE_MS`.

## 6. Valve position control

- [x] 6.1 Implement the PID controller for POSITION mode, driving toward the D2-derived target through the interlock's drive burst; verify on the bench that the valve converges to and holds a commanded mid-range target within `POSITION_DEADBAND`.
- [x] 6.2 Implement the dead-reckoning estimator for TIME mode (per-tick accumulation using `t_open_ms`/`t_close_ms` depending on the active direction, clamped to [0,1]); verify with a native unit test covering forward accumulation, a direction reversal mid-travel, and clamping at 0/1.
- [x] 6.3 Implement the one-shot seating push at mechanical stops (see design.md Decision 16): the moment the valve is first confirmed at a stop matching the current target, drive at `SEAT_KICK_DUTY_PERCENT` for `SEAT_KICK_MS`, then release the motor, for both POSITION and TIME modes; verify on the bench that the push fires once per arrival (not repeatedly while seated) and that the valve stays put unpowered afterward, without being nudged by hand.
- [ ] 6.5 Implement mid-travel overcurrent fault detection (stall current exceeded while not confirmed at a recorded stop) using `CurrentSensing`'s filtered reading at `CURRENT_SENSE_FILTER_WINDOW_MS` (see design.md Decision 17), stopping the motor immediately and triggering homing-calibration's re-homing; verify on the bench by obstructing the valve mid-travel and confirming an immediate stop, a fault log entry, and a subsequent re-homing sweep, with a stable reading through PWM ripple.
- [x] 6.6 Implement the fail-safe on D2 signal loss (force target = fully open, reuse the seating-push-at-stops path) and resume-on-recovery; verify on the bench by disconnecting the D2 signal mid-operation and confirming the valve drives to and seats at open, then resumes tracking the ECU target once the signal returns.
- [x] 6.7 Implement the position-deadband guard that suppresses motor engagement for small deviations; verify with a native unit test confirming no drive decision is produced for deviations within `POSITION_DEADBAND`.

## 7. Diagnostics and logging

- [x] 7.1 Implement `Serial.begin(115200)` at startup and the always-on event logger (homing start/end, mode selection, faults, fail-safe entry/exit); verify by reviewing serial output across a full bench run that covers each event type at least once.
- [x] 7.2 Implement throttled telemetry logging (target, position/estimate, drive state, current, battery voltage) at `TELEMETRY_LOG_INTERVAL_MS`; verify from serial output timestamps that telemetry lines are spaced at least the configured interval apart even under continuous operation.
- [x] 7.3 Audit that no serial writes occur inside the interlock's read window (task 3.2); verify with a temporary debug guard that trips if `Serial.print` is called while the window is open, confirm it never trips across a bench run, then remove the guard.

## 8. Integration

- [x] 8.1 Wire all modules together in `src/main.cpp` (HOMING -> RUNNING state machine, with the orthogonal DRIVE/READ interlock cycle running throughout) with no blocking calls anywhere in `loop()`; verify with a successful `pio run` build and a full bench run from power-on through homing into steady-state control.
- [ ] 8.2 Run the full bench acceptance pass described in `design.md`'s Migration Plan step 3 (motor/pot on the bench, ECU signal simulated): confirm homing, both control modes (forcing TIME mode by covering/disconnecting the pot if needed), the seating push at both stops, the mid-travel fault plus re-homing, and the D2 signal-loss fail-safe all behave as specified; record which placeholder constants in `include/config.h` were adjusted during this pass.
