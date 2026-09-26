#include "CurrentSensing.h"
#include "Pins.h"
#include "config.h"
#include <CurrentSense.h>

namespace
{
    float readAveragedAmps(uint8_t pin, unsigned long windowMs)
    {
        long sum = 0;
        unsigned int count = 0;
        const unsigned long start = millis();
        do
        {
            sum += analogRead(pin);
            count++;
        } while (millis() - start < windowMs);
        return adcCountsToAmps((int)(sum / (long)count));
    }
}

namespace CurrentSensing
{
    float readRAmps(unsigned long windowMs)
    {
        return readAveragedAmps(pins::CURRENT_R_IS, windowMs);
    }

    float readLAmps(unsigned long windowMs)
    {
        return readAveragedAmps(pins::CURRENT_L_IS, windowMs);
    }

    float readActiveDirectionAmps(ActuatorInterlock::Direction direction, unsigned long windowMs)
    {
        switch (direction)
        {
        case ActuatorInterlock::Direction::Opening:
            return readRAmps(windowMs);
        case ActuatorInterlock::Direction::Closing:
            return readLAmps(windowMs);
        default:
            return 0.0f;
        }
    }

    float readBatteryVolts()
    {
        const int raw = analogRead(pins::BATTERY_VOLTAGE);
        const float pinVolts = (float)raw / cfg::ADC_MAX_COUNTS * cfg::ADC_REF_VOLTS;
        return pinVolts * cfg::BATTERY_VOLTAGE_DIVIDER_RATIO;
    }
}
