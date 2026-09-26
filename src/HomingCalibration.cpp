#include "HomingCalibration.h"
#include "ActuatorInterlock.h"
#include "CurrentSensing.h"
#include "Logger.h"
#include "config.h"

namespace
{
    using ActuatorInterlock::Direction;

    // Drives toward `direction`, detecting the mechanical stop with a
    // baseline of the active direction's current learned once per drive,
    // instead of a fixed Amp threshold (see design.md Decisions 11-15) - so
    // the trigger level is referenced to that direction's own current each
    // run rather than a hand-picked Amp value that has to be re-tuned per
    // unit/wear/battery voltage. Every reading below comes from
    // CurrentSensing already block-averaged over
    // CURRENT_SENSE_HARD_LIMIT_WINDOW_MS (see design.md Decision 17), never
    // a single raw sample.
    //
    // The whole sweep runs in well under 1 second on this hardware (Decision
    // 15), so the baseline-learning window has to be short enough to finish
    // BEFORE the valve can reach a stop:
    // - HOMING_CURRENT_HARD_LIMIT_AMPS is checked against every reading,
    //   every iteration - an absolute backstop that must not wait for the
    //   baseline/spike logic below.
    // - The first HOMING_CURRENT_INRUSH_IGNORE_MS is skipped (clears the
    //   ~2ms BTS7960 direction-reversal dead-time, a real zero-current
    //   window), then the next HOMING_CURRENT_BASELINE_LEARN_MS of readings
    //   are averaged into ONE baseline value, logged, and held fixed for the
    //   rest of this drive - deliberately not an ongoing EMA (Decision 13),
    //   since an EMA whose own gating condition depends on itself is a
    //   feedback loop with no guarantee of converging on the true running
    //   current.
    // - A reading at or above baseline * SPIKE_FACTOR, floored at
    //   SPIKE_MIN_ABS_AMPS (Decision 14), is a candidate spike; it must stay
    //   at or above that line for SPIKE_SUSTAIN_MS to confirm a stop,
    //   rejecting a single noisy sample.
    // - HOMING_MAX_DRIVE_MS remains a time-based backstop if no current
    //   check ever fires (e.g. a disconnected current-sense wire).
    //
    // Returns the elapsed time up to the moment the stall was first detected
    // - for the dynamic spike path, that's the spike's onset
    // (`spikeStartMs`), not the later instant `HOMING_CURRENT_SPIKE_SUSTAIN_MS`
    // finishes confirming it, so the confirmation wait doesn't inflate the
    // reported drive time. The hard-limit and timeout paths have no such
    // confirmation delay to exclude.
    unsigned long driveToStop(Direction direction)
    {
        const unsigned long start = millis();

        ActuatorInterlock::drive(direction, cfg::HOMING_DRIVE_DUTY_PERCENT);

        float learnSum = 0.0f;
        unsigned int learnCount = 0;
        unsigned long learnStartMs = 0;

        float baseline = -1.0f;
        float spikeThreshold = 0.0f;
        unsigned long spikeStartMs = 0;
        unsigned long lastLogMs = start;
        bool stopped = false;
        unsigned long elapsed = 0;

        // Diagnostic: tracks how often an above-threshold streak gets reset
        // before reaching HOMING_CURRENT_SPIKE_SUSTAIN_MS, and the longest
        // streak reached, to check whether the 1ms block-average window is
        // too close to the ~1ms PWM period to hold a reading above threshold
        // continuously (see design.md Decision 17's last risk note).
        unsigned int spikeResetCount = 0;
        unsigned long spikeMaxStreakMs = 0;

        while ((millis() - start) < cfg::HOMING_MAX_DRIVE_MS)
        {
            const float reading = CurrentSensing::readActiveDirectionAmps(direction, cfg::CURRENT_SENSE_HARD_LIMIT_WINDOW_MS);
            const unsigned long now = millis();

            if (reading >= cfg::HOMING_CURRENT_HARD_LIMIT_AMPS)
            {
                Logger::event(F("homing: hard current limit reached, stopping"));
                stopped = true;
                elapsed = now - start;
                break;
            }

            if (now - lastLogMs > 100)
            {
                Serial.print(now - start);
                Serial.print(F("ms "));
                Serial.print(reading);
                Serial.print(F(" baseline="));
                Serial.print(baseline);
                Serial.print(F(" threshold="));
                Serial.println(baseline < 0.0f ? -1.0f : spikeThreshold);
                lastLogMs = now;
            }

            if (now - start < cfg::HOMING_CURRENT_INRUSH_IGNORE_MS)
            {
                continue;
            }

            if (baseline < 0.0f)
            {
                if (learnStartMs == 0)
                {
                    learnStartMs = now;
                }

                learnSum += reading;
                learnCount++;

                if (now - learnStartMs >= cfg::HOMING_CURRENT_BASELINE_LEARN_MS)
                {
                    baseline = learnSum / (float)learnCount;
                    const float relativeThreshold = baseline * cfg::HOMING_CURRENT_SPIKE_FACTOR;
                    spikeThreshold = relativeThreshold > cfg::HOMING_CURRENT_SPIKE_MIN_ABS_AMPS ? relativeThreshold : cfg::HOMING_CURRENT_SPIKE_MIN_ABS_AMPS;
                    Logger::event(String(F("homing: baseline locked at ")) + baseline + F(" threshold=") + spikeThreshold);
                }
                continue;
            }

            if (reading >= spikeThreshold)
            {
                if (spikeStartMs == 0)
                {
                    spikeStartMs = now;
                }
                else if (now - spikeStartMs >= cfg::HOMING_CURRENT_SPIKE_SUSTAIN_MS)
                {
                    stopped = true;
                    elapsed = spikeStartMs - start;
                    Logger::event(String(F("homing: spike confirmed, elapsed=")) + (now - start) +
                                  F(" spikeOnsetElapsed=") + elapsed);
                    break;
                }
            }
            else
            {
                if (spikeStartMs != 0)
                {
                    const unsigned long streakMs = now - spikeStartMs;
                    if (streakMs > spikeMaxStreakMs)
                    {
                        spikeMaxStreakMs = streakMs;
                    }
                    spikeResetCount++;
                }
                spikeStartMs = 0;
            }
        }

        Logger::event(String(F("homing: spike resets=")) + spikeResetCount + F(" maxStreakMs=") + spikeMaxStreakMs);

        if (!stopped)
        {
            Logger::event(F("homing: timed out waiting for stall current, stopping anyway"));
            elapsed = millis() - start;
        }

        ActuatorInterlock::stopMotor();
        return elapsed;
    }

    // Unconditional fixed-duration opening kick, with no stall-current check,
    // run before every sweep regardless of where the valve currently rests.
    // Releases the valve if it's already at the closed stop, so the closing
    // leg's stall detection can't fire at t=0 and report ~0ms (see
    // design.md Decision 10).
    void preOpenKick()
    {
        ActuatorInterlock::drive(Direction::Opening, cfg::HOMING_PRE_OPEN_KICK_DUTY_PERCENT);
        delay(cfg::HOMING_PRE_OPEN_KICK_MS);
        ActuatorInterlock::stopMotor();
    }

    // Trims driveToStop()'s measured elapsed time down by a bench-tuned
    // percentage (see design.md Decision 18 and the
    // HOMING_CLOSE/OPEN_TIME_COMPENSATION_PERCENT comment in config.h).
    unsigned long applyStopTimeCompensation(unsigned long measuredMs, float compensationPercent)
    {
        const float factor = (100.0f - compensationPercent) / 100.0f;
        return (unsigned long)((float)measuredMs * factor);
    }
}

namespace HomingCalibration
{
    Result run()
    {
        Logger::event(F("homing: start"));

        if (cfg::PID_BENCH_TEST_MODE)
        {
            Logger::event(F("homing: bench test mode, skipping drive, using fixed calibration"));
            return Result{cfg::PID_BENCH_TEST_CLOSED_RAW, cfg::PID_BENCH_TEST_OPEN_RAW, 0, 0, ControlMode::Position};
        }

        Logger::event(String(F("homing: pre open kick")));
        preOpenKick();

        // delay to estabilize the current
        delay(100);

        // Fixed convention: close first, then open (arbitrary but fixed -
        // see design.md).

        Logger::event(String(F("homing: driving to close")));
        const unsigned long closeTimeMsRaw = driveToStop(Direction::Closing);
        const unsigned long closeTimeMs = applyStopTimeCompensation(closeTimeMsRaw, cfg::HOMING_CLOSE_TIME_COMPENSATION_PERCENT);
        const int closedStopRaw = ActuatorInterlock::readPositionRaw();

        Logger::event(String(F("homing: driving to open")));
        const unsigned long openTimeMsRaw = driveToStop(Direction::Opening);
        const unsigned long openTimeMs = applyStopTimeCompensation(openTimeMsRaw, cfg::HOMING_OPEN_TIME_COMPENSATION_PERCENT);
        const int openStopRaw = ActuatorInterlock::readPositionRaw();

        const ControlMode mode = selectControlMode(closedStopRaw, openStopRaw, cfg::POSITION_MIN_RANGE_COUNTS);

        const Result result{closedStopRaw, openStopRaw, openTimeMs, closeTimeMs, mode};

        String msg = String(F("homing: done closedRaw=")) + closedStopRaw +
                     F(" openRaw=") + openStopRaw +
                     F(" closeTimeMs=") + closeTimeMs + F(" (raw=") + closeTimeMsRaw + F(")") +
                     F(" openTimeMs=") + openTimeMs + F(" (raw=") + openTimeMsRaw + F(")") +
                     F(" mode=") + (mode == ControlMode::Position ? F("POSITION") : F("TIME"));
        Logger::event(msg);

        return result;
    }
}
