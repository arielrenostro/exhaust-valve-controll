#ifndef EXHAUST_VALVE_CONFIG_H
#define EXHAUST_VALVE_CONFIG_H

// Centralized tunable constants for the exhaust valve controller.
// Every value here is a bench-calibration placeholder (see design.md's
// Migration Plan step 3) - nothing here is derived from a spec requirement,
// only the requirements themselves are normative.
//
// This header has no Arduino.h dependency so it can be included from both
// the embedded build (env:nanoatmega328new) and native unit tests
// (env:native).

#include <stdint.h>

namespace cfg
{
    // Pin assignments live in include/Pins.h (it needs Arduino.h for the A0-A3
    // macros, so it is kept separate from this Arduino-independent header).

    // ---- Drive/read interlock timing (specs/exhaust-valve/actuator-interlock) ----
    constexpr unsigned long T_DRIVE_MS = 20; // length of each motor drive burst

    // Two different components turn off, so two different dead-times:
    //
    // Time to let the BTS7960's own R_PWM/L_PWM outputs actually finish
    // turning off before the other side of the shared wire is energized -
    // used before D9 enables (motor -> read) and before applying a reversed
    // direction (both are "wait for the BTS7960 output to truly be off").
    // Per the user's reading of the BTS7960 datasheet, 2ms is a comfortable
    // margin over its turn-off delay.
    constexpr unsigned long T_BTS7960_TURNOFF_MS = 2;

    // Time to let the D9 read-enable MOSFET (2N7000) actually finish turning
    // off before the motor re-energizes (read -> motor). The 2N7000
    // datasheet gives a typical switching time of ~10ns; this default is
    // ~2000x that for margin (the datasheet figure is the bare transistor
    // under specific test conditions, not necessarily identical to being
    // driven directly off an Arduino pin with no dedicated gate driver).
    // In microseconds, not milliseconds, since it's this much smaller than
    // T_BTS7960_TURNOFF_MS - see settleAfterReadMosfetOff() in
    // ActuatorInterlock.cpp.
    constexpr unsigned long T_READ_MOSFET_TURNOFF_US = 20;

    // ---- ADC (position + current sensing) ----
    // Prescaler 32 -> ~500kHz ADC clock, ~26-28us conversion. Traded down from
    // the default prescaler 128 (full accuracy, ~104us) because the measured
    // sensor range (~443 counts) leaves ample margin over
    // POSITION_MIN_RANGE_COUNTS. Raise back toward 128 if a narrower-range
    // sensor is ever used.
    constexpr uint8_t ADC_PRESCALER = 32;
    constexpr float ADC_REF_VOLTS = 5.0f;
    constexpr float ADC_MAX_COUNTS = 1023.0f;

    // ---- ECU PWM target input (specs/exhaust-valve/pwm-target-input) ----
    constexpr unsigned long D2_SIGNAL_TIMEOUT_MS = 100; // ~10 periods of the 100Hz ECU signal
    constexpr bool INVERT_TARGET_DUTY = true;          // 0%=closed/100%=open confirmed for this ECU; flip if a different unit uses the opposite convention

    // ---- Bench target simulation (no PWM signal generator available yet) ----
    // When true, TargetSource reads a potentiometer on A4 instead of
    // capturing the real ECU PWM on D2. Flip to false once a real PWM
    // source (bench generator or the actual ECU) is available.
    constexpr bool SIMULATE_TARGET_WITH_POTENTIOMETER = true;

    // The bench potentiometer's usable analog range maps to 0-100% duty;
    // 900 (not the full 0-1023 ADC range) matches this potentiometer as
    // wired, same idea as the position sensor not spanning the full range.
    constexpr int TARGET_SIM_POT_MAX_COUNTS = 900;

    // ---- Current-sense scaling (BTS7960 R_IS/L_IS) ----
    // Linear scale measured on this board: 0.00V = 0A, 4.0V = 3.4A.
    constexpr float CURRENT_SENSE_VOLTS_AT_MAX = 4.0f;
    constexpr float CURRENT_SENSE_AMPS_AT_MAX = 3.4f;

    // Raw current samples are dominated by PWM-frequency ripple (D5/D6 run
    // at the Nano's default ~980Hz, with peaks comparable in magnitude to a
    // genuine stall), so CurrentSensing::readRAmps/readLAmps block-average
    // over one of these caller-supplied windows instead of returning a
    // single raw sample. FILTER (a handful of PWM periods) is used for
    // comparisons outside of homing's driveToStop() - mid-travel fault,
    // telemetry; HARD_LIMIT is a shorter window (~1 PWM period) used
    // throughout driveToStop() itself (hard-limit check and baseline/spike
    // detection alike), which needs to react faster than a full sweep's
    // time budget allows for.
    constexpr unsigned long CURRENT_SENSE_FILTER_WINDOW_MS = 5;
    constexpr unsigned long CURRENT_SENSE_HARD_LIMIT_WINDOW_MS = 1;

    // Stall/batente detection current, per direction (spring/gravity/exhaust
    // pressure can assist one direction over the other).
    constexpr float STALL_CURRENT_THRESHOLD_R_AMPS = 4.0f;
    constexpr float STALL_CURRENT_THRESHOLD_L_AMPS = 4.0f;

    constexpr uint8_t MOTOR_DRIVE_DUTY_PERCENT = 40; // fixed duty used for open-loop (TIME mode) driving

    // ---- Homing (specs/exhaust-valve/homing-calibration) ----
    // Independent of MOTOR_DRIVE_DUTY_PERCENT on purpose: homing only runs
    // briefly (boot / re-home), so it can afford a higher duty than
    // continuous normal-operation driving, which gives a clearer
    // running-vs-stalled current gap for stall detection (see design.md
    // Decision 12).
    constexpr uint8_t HOMING_DRIVE_DUTY_PERCENT = MOTOR_DRIVE_DUTY_PERCENT; // motor duty applied while sweeping to find each stop
    constexpr uint16_t POSITION_MIN_RANGE_COUNTS = 100; // below this, control falls back to time-based mode

    // Safety guard beyond the literal spec text: if a stall current is never
    // detected (e.g. a disconnected current-sense wire), stop driving after
    // this long instead of running the motor indefinitely into a fault. A
    // full sweep normally completes in well under 1 second (see design.md
    // Decision 15), so this only needs to be a short backstop.
    constexpr unsigned long HOMING_MAX_DRIVE_MS = 1500;

    // Unconditional opening kick applied before the close-then-open sweep,
    // with no stall-current check during it (see design.md Decision 10).
    // Fixes closeTimeMs coming back near-zero when the valve is already
    // resting at the closed stop when homing starts.
    constexpr unsigned long HOMING_PRE_OPEN_KICK_MS = 250;
    constexpr uint8_t HOMING_PRE_OPEN_KICK_DUTY_PERCENT = 80;

    // Dynamic, per-run baseline-relative stall detection for driveToStop()
    // only - replaces a fixed Amp threshold with one referenced to that
    // direction's own current level each run, so it isn't a hand-picked
    // value that has to be re-tuned per unit/wear/battery voltage. Does NOT
    // affect STALL_CURRENT_THRESHOLD_R/L_AMPS above, which still govern
    // valve-position-control's mid-travel fault check. Every reading it
    // consumes is already block-averaged by CurrentSensing over
    // CURRENT_SENSE_HARD_LIMIT_WINDOW_MS (see design.md Decision 17).
    //
    // A full stop-to-stop sweep on this hardware runs in well under 1
    // second, so the baseline-learning window below has to be short enough
    // to finish before the valve can reach a stop (see design.md Decision
    // 15).

    // The baseline is learned once per drive and then held fixed for the
    // rest of it, rather than continuously updated via an EMA (see design.md
    // Decision 13): an EMA whose own gating condition depends on itself is a
    // feedback loop with no guarantee of converging on the true running
    // current.
    //
    // HOMING_CURRENT_INRUSH_IGNORE_MS only needs to clear the high current 
    // from start moving (a real zero-current window, not motor
    // behavior) before HOMING_CURRENT_BASELINE_LEARN_MS starts averaging
    // filtered samples into the baseline - both are short because the whole
    // sweep is short: there is no mechanical ramp-up to wait out on this
    // hardware. The locked baseline is logged once via Logger::event for
    // visibility.
    constexpr unsigned long HOMING_CURRENT_INRUSH_IGNORE_MS = 20;
    constexpr unsigned long HOMING_CURRENT_BASELINE_LEARN_MS = 40;
    constexpr float HOMING_CURRENT_SPIKE_FACTOR = 1.2f;
    constexpr float HOMING_CURRENT_SPIKE_MIN_ABS_AMPS = 0.25f;
    constexpr unsigned long HOMING_CURRENT_SPIKE_SUSTAIN_MS = 5; // how long the spike must persist to count as a real stall and not a single noisy sample - short, since the whole sweep runs in well under 1s
    constexpr float HOMING_CURRENT_HARD_LIMIT_AMPS = 2.0f;       // absolute backstop: stop immediately at/above this regardless of the dynamic baseline

    // Bench investigation (design.md Decision 18) ruled out spike-sustain
    // overhead, baseline-learning bias, and 1ms-window aliasing as causes of
    // a large, fairly consistent gap between driveToStop()'s measured stop
    // time and the valve's actual physical stop - current evidence points to
    // mechanical slack between the motor and the valve that no current-based
    // detection can see. These trim that percentage off closeTimeMs/openTimeMs
    // before they're used as the TIME-mode dead-reckoning speed reference, so
    // the estimate reaches each extreme when the valve physically does,
    // instead of only when driveToStop() eventually confirms the stall. Per
    // direction, since the observed gap isn't symmetric; bench-tune
    // independently of each other.
    constexpr float HOMING_CLOSE_TIME_COMPENSATION_PERCENT = 15.0f;
    constexpr float HOMING_OPEN_TIME_COMPENSATION_PERCENT = 15.0f;

    // ---- Position control (specs/exhaust-valve/valve-position-control) ----
    // Position and target are normalized to a 0.0 (closed) - 1.0 (open)
    // fraction of full travel in both control modes, so the same deadband/PID
    // constants apply regardless of whether position comes from the sensor or
    // from dead-reckoning.
    constexpr float POSITION_DEADBAND = 0.02f; // 2% of full travel: no-drive band, and how close counts as "confirmed at a stop" to enter holding

    // Wider than POSITION_DEADBAND on purpose: used only to tell an expected
    // stall while arriving at a mechanical stop apart from a genuine
    // mid-travel obstruction (see design note in ValvePositionControl.cpp).
    // Needs to be generous enough to absorb TIME-mode dead-reckoning drift
    // over one full traversal.
    constexpr float STOP_PROXIMITY_BAND = 0.15f;

    constexpr float PID_KP = 2.0f;
    constexpr float PID_KI = 0.0f;
    constexpr float PID_KD = 0.1f;

    // One-shot confirmation push applied the moment the valve is first
    // confirmed at a mechanical stop (see design.md Decision 16), instead of
    // continuous holding force: the actuator's drive mechanism is
    // non-back-drivable, so once fully seated it stays put without power,
    // and only needs a firm push to make sure it's actually against the
    // stop rather than just within the position deadband.
    constexpr unsigned long SEAT_KICK_MS = 100;
    constexpr uint8_t SEAT_KICK_DUTY_PERCENT = 50;

    // ---- Battery voltage sensing (A3) ----
    // Simple resistor divider: R1 = 30100 ohm (top, to battery), R2 = 10000
    // ohm (bottom, to GND), pin taken across R2. Measured/nominal: 20.05V
    // battery -> 5.00V at the pin, i.e. ratio (R1+R2)/R2 = 40100/10000 = 4.01.
    constexpr float BATTERY_VOLTAGE_DIVIDER_RATIO = 4.01f;

    // ---- Diagnostics logging (specs/exhaust-valve/diagnostics-logging) ----
    constexpr unsigned long TELEMETRY_LOG_INTERVAL_MS = 300;
}

#endif
