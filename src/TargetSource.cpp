#include "TargetSource.h"
#include "Pins.h"
#include "config.h"
#include <PwmTargetReader.h>
#include <AnalogTargetMapping.h>

namespace TargetSource
{
    void begin()
    {
        if (cfg::SIMULATE_TARGET_WITH_POTENTIOMETER)
        {
            pinMode(pins::TARGET_SIM_POTENTIOMETER, INPUT);
        }
        else
        {
            PwmTargetReader::begin(pins::ECU_PWM_TARGET, cfg::D2_SIGNAL_TIMEOUT_MS);
        }
    }

    float getDutyPercent()
    {
        if (cfg::SIMULATE_TARGET_WITH_POTENTIOMETER)
        {
            const int raw = analogRead(pins::TARGET_SIM_POTENTIOMETER);
            return analogCountsToPercent(raw, cfg::TARGET_SIM_POT_MAX_COUNTS);
        }
        return PwmTargetReader::getDutyPercent();
    }

    bool isValid()
    {
        if (cfg::SIMULATE_TARGET_WITH_POTENTIOMETER)
        {
            return true;
        }
        return PwmTargetReader::isValid();
    }
}
