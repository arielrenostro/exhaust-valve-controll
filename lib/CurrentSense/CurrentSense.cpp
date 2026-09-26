#include "CurrentSense.h"
#include "config.h"

float adcCountsToAmps(int adcCounts)
{
    const float volts = (float)adcCounts / cfg::ADC_MAX_COUNTS * cfg::ADC_REF_VOLTS;
    return volts / cfg::CURRENT_SENSE_VOLTS_AT_MAX * cfg::CURRENT_SENSE_AMPS_AT_MAX;
}
