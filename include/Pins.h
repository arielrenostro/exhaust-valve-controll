#ifndef EXHAUST_VALVE_PINS_H
#define EXHAUST_VALVE_PINS_H

// Fixed hardware pin assignments (not calibration - see config.h for tunable
// values). Requires Arduino.h for the A0-A3 macros, so only embedded
// (env:nanoatmega328new) source files include this, never native tests.

#include <Arduino.h>

namespace pins
{
    constexpr uint8_t ECU_PWM_TARGET = 2;   // D2: ECU PWM duty target (input, interrupt)
    constexpr uint8_t MOTOR_R_PWM = 6;      // D5: BTS7960 R_PWM
    constexpr uint8_t MOTOR_L_PWM = 5;      // D6: BTS7960 L_PWM
    constexpr uint8_t MOTOR_EN = 7;         // D7: BTS7960 R_EN + L_EN (tied together)
    constexpr uint8_t POSITION_READ_EN = 9; // D9: MOSFET enabling A0 for position read

    constexpr uint8_t POSITION_SENSE = A0;   // A0: potentiometer position sensor (shared wire with motor)
    constexpr uint8_t CURRENT_R_IS = A2;     // A1: BTS7960 R_IS current sense
    constexpr uint8_t CURRENT_L_IS = A1;     // A2: BTS7960 L_IS current sense
    constexpr uint8_t BATTERY_VOLTAGE = A3;  // A3: car battery voltage sense

    // A4: bench-only potentiometer standing in for the ECU's PWM target
    // signal until a real PWM source is available - see
    // cfg::SIMULATE_TARGET_WITH_POTENTIOMETER and TargetSource.
    constexpr uint8_t TARGET_SIM_POTENTIOMETER = A4;
}

#endif
