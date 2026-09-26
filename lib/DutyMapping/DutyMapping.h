#ifndef DUTY_MAPPING_H
#define DUTY_MAPPING_H

// Maps a measured ECU PWM duty cycle (0-100%, may be out of range for a
// noisy/marginal reading) to a requested valve opening target expressed as a
// fraction of full travel: 0.0 = fully closed, 1.0 = fully open.
// See specs/exhaust-valve/pwm-target-input - Duty-to-target mapping.
float dutyPercentToTargetFraction(float dutyPercent);

#endif
