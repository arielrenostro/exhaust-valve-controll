#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

// Estimates valve position (0.0 = closed, 1.0 = open) by accumulating
// elapsed drive time scaled by the calibrated opening/closing speed for
// whichever direction is currently active. Used in TIME control mode.
// See specs/exhaust-valve/valve-position-control - Open-loop time-based
// control.
class DeadReckoningEstimator
{
public:
    DeadReckoningEstimator(unsigned long openTimeMs, unsigned long closeTimeMs, float initialFraction = 0.0f);

    // direction: > 0 = opening, < 0 = closing, 0 = stopped (no change).
    void update(int direction, unsigned long elapsedMs);

    // Forces the estimate to an exact value, used when a stop is physically
    // confirmed by current-based stall detection (bounds drift).
    void resyncTo(float fraction);

    float fraction() const;

private:
    unsigned long m_openTimeMs;
    unsigned long m_closeTimeMs;
    float m_fraction;
};

#endif
