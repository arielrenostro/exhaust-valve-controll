#ifndef ANALOG_TARGET_MAPPING_H
#define ANALOG_TARGET_MAPPING_H

// Maps a raw ADC reading from the bench target-simulation potentiometer
// (A4) onto a duty percentage in [0,100], using `maxCounts` as the reading
// that represents 100% (not necessarily the full 0-1023 ADC range - see
// cfg::TARGET_SIM_POT_MAX_COUNTS). Values outside [0, maxCounts] clamp.
float analogCountsToPercent(int rawCounts, int maxCounts);

#endif
