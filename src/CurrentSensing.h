#ifndef CURRENT_SENSING_H
#define CURRENT_SENSING_H

#include <Arduino.h>
#include "ActuatorInterlock.h"

// Hardware-facing current and battery-voltage sensing (A1/A2/A3). Unlike
// A0, these pins are not part of the shared/interlocked wire and can be
// sampled at any time, including while the motor is driving.
namespace CurrentSensing
{
    // NOTE: Opening is wired to R_PWM/R_IS and Closing to L_PWM/L_IS as the
    // implementation convention (the specs deliberately leave which side is
    // which unspecified). If bench testing shows the valve opens when L is
    // driven instead, swap the two branches in ActuatorInterlock::drive() and
    // the two functions below together.
    //
    // Each reading is block-averaged over windowMs to ride out D5/D6's
    // PWM-frequency ripple on the BTS7960 current-sense outputs; callers pick
    // the window (see config.h's CURRENT_SENSE_FILTER_WINDOW_MS /
    // CURRENT_SENSE_HARD_LIMIT_WINDOW_MS) that fits their reaction-time
    // budget.
    float readRAmps(unsigned long windowMs);
    float readLAmps(unsigned long windowMs);

    // Current sense reading for whichever direction is currently being
    // driven (0A if Stopped).
    float readActiveDirectionAmps(ActuatorInterlock::Direction direction, unsigned long windowMs);

    float readBatteryVolts();
}

#endif
