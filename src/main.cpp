#include <Arduino.h>
#include "config.h"
#include "Pins.h"
#include "TargetSource.h"
#include "ActuatorInterlock.h"
#include "CurrentSensing.h"
#include "Logger.h"
#include "HomingCalibration.h"
#include "ValvePositionControl.h"

void setup()
{
    Logger::begin();
    Logger::event(cfg::SIMULATE_TARGET_WITH_POTENTIOMETER
                       ? F("target source: A4 potentiometer (bench simulation)")
                       : F("target source: D2 PWM (ECU)"));
    TargetSource::begin();
    ActuatorInterlock::begin();

    const HomingCalibration::Result calibration = HomingCalibration::run();
    ValvePositionControl::reset(calibration);
}

void loop()
{
    if (ValvePositionControl::update())
    {
        // Mid-travel overcurrent fault: discard the old calibration and
        // re-home before resuming control (specs/exhaust-valve/homing-calibration
        // - Re-homing after mid-travel fault).
        const HomingCalibration::Result calibration = HomingCalibration::run();
        ValvePositionControl::reset(calibration);
    }
}
