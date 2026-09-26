#ifndef PID_H
#define PID_H

// Minimal PID controller operating on a normalized error (target - current
// position, both fractions of full travel in [0,1]). Output is clamped to
// [-1, 1]; the caller maps sign to drive direction and magnitude to a duty
// percentage. See specs/exhaust-valve/valve-position-control - Closed-loop
// position control.
class PidController
{
public:
    PidController(float kp, float ki, float kd);

    void reset();

    // dtSeconds should be > 0; derivative term is skipped on the first call
    // after construction/reset.
    float update(float error, float dtSeconds);

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_integral;
    float m_previousError;
    bool m_hasPreviousError;
};

#endif
