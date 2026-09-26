#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

// Converts a raw ADC reading (0-1023) from a BTS7960 current-sense pin
// (R_IS/L_IS) into Amps, using the board's measured linear scale
// (0.00V = 0A, 4.0V = 3.4A). See design.md Context - current-sense scaling.
float adcCountsToAmps(int adcCounts);

#endif
