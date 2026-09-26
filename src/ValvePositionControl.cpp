#include "ValvePositionControl.h"
#include "ActuatorInterlock.h"
#include "CurrentSensing.h"
#include "Logger.h"
#include "config.h"
#include "TargetSource.h"
#include <DutyMapping.h>
#include <Pid.h>
#include <DeadReckoning.h>
#include <PositionDeadband.h>

namespace
{
    using ActuatorInterlock::Direction;

    HomingCalibration::Result s_calibration;
    PidController s_pid(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD);
    DeadReckoningEstimator s_estimator(0, 0, 0.0f);

    enum class Phase
    {
        AwaitingTick,
        Driving
    };

    Phase s_phase = Phase::AwaitingTick;
    unsigned long s_tickStartMs = 0;
    unsigned long s_activeDriveDurationMs = cfg::T_DRIVE_MS;
    Direction s_activeDirection = Direction::Stopped;
    bool s_activeIsSeating = false;
    float s_positionAtTickStart = 0.0f;

    // Which extreme (if any) already received its one-shot seating push;
    // Stopped means neither. Cleared whenever the valve is commanded away
    // from that extreme, so the next arrival gets a fresh push.
    Direction s_seatedAt = Direction::Stopped;

    float s_lastTargetFraction = 0.0f;
    float s_lastCurrentAmps = 0.0f;

    int directionSign(Direction d)
    {
        return d == Direction::Opening ? 1 : (d == Direction::Closing ? -1 : 0);
    }

    float clamp01(float v)
    {
        if (v < 0.0f)
        {
            return 0.0f;
        }
        if (v > 1.0f)
        {
            return 1.0f;
        }
        return v;
    }

    float stallThresholdFor(Direction direction)
    {
        return direction == Direction::Opening
                   ? cfg::STALL_CURRENT_THRESHOLD_R_AMPS
                   : cfg::STALL_CURRENT_THRESHOLD_L_AMPS;
    }

    // POSITION mode only: maps a raw A0 reading onto the [0,1] fraction of
    // travel established by homing. Assumes closedRaw != openRaw, which
    // POSITION mode guarantees (selectControlMode requires a >= 100-count
    // range to pick POSITION mode).
    float mapRawToFraction(int raw)
    {
        const float fraction = (float)(raw - s_calibration.closedStopRaw) /
                                (float)(s_calibration.openStopRaw - s_calibration.closedStopRaw);
        return clamp01(fraction);
    }

    float readCurrentPositionFraction()
    {
        if (s_calibration.mode == ControlMode::Position)
        {
            return mapRawToFraction(ActuatorInterlock::readPositionRaw());
        }
        return s_estimator.fraction();
    }

    float readTargetFraction()
    {
        if (!TargetSource::isValid())
        {
            // Fail-safe: drive to and seat at fully open until the ECU
            // signal returns (see specs/exhaust-valve/valve-position-control
            // - Fail-safe on signal loss). Reuses the same seating-at-stops
            // path as a normal fully-open command - no separate fail-safe
            // code path.
            return 1.0f;
        }
        return dutyPercentToTargetFraction(TargetSource::getDutyPercent());
    }

    bool isConfirmedAtOpenStop(float position)
    {
        return position >= (1.0f - cfg::POSITION_DEADBAND);
    }

    bool isConfirmedAtClosedStop(float position)
    {
        return position <= cfg::POSITION_DEADBAND;
    }

    bool isNearOpenStop(float position)
    {
        return position >= (1.0f - cfg::STOP_PROXIMITY_BAND);
    }

    bool isNearClosedStop(float position)
    {
        return position <= cfg::STOP_PROXIMITY_BAND;
    }

    void decideAndStartTick()
    {
        const float position = readCurrentPositionFraction();
        const float target = readTargetFraction();

        Direction direction = Direction::Stopped;
        uint8_t duty = 0;
        bool seating = false;
        unsigned long driveDurationMs = cfg::T_DRIVE_MS;

        const bool targetIsOpen = target >= (1.0f - cfg::POSITION_DEADBAND);
        const bool targetIsClosed = target <= cfg::POSITION_DEADBAND;

        if (targetIsOpen && isConfirmedAtOpenStop(position))
        {
            if (s_seatedAt != Direction::Opening)
            {
                direction = Direction::Opening;
                duty = cfg::SEAT_KICK_DUTY_PERCENT;
                driveDurationMs = cfg::SEAT_KICK_MS;
                seating = true;
            }
        }
        else if (targetIsClosed && isConfirmedAtClosedStop(position))
        {
            if (s_seatedAt != Direction::Closing)
            {
                direction = Direction::Closing;
                duty = cfg::SEAT_KICK_DUTY_PERCENT;
                driveDurationMs = cfg::SEAT_KICK_MS;
                seating = true;
            }
        }
        else if (shouldDrive(position, target, cfg::POSITION_DEADBAND))
        {
            // Commanded away from wherever it was seated: the next arrival
            // at an extreme (this one or the other) needs a fresh push.
            s_seatedAt = Direction::Stopped;

            if (s_calibration.mode == ControlMode::Position)
            {
                const float error = target - position;
                const float output = s_pid.update(error, (float)cfg::T_DRIVE_MS / 1000.0f);
                direction = output > 0.0f ? Direction::Opening : Direction::Closing;
                const float magnitude = output < 0.0f ? -output : output;
                duty = (uint8_t)(magnitude * 100.0f);
                if (duty == 0)
                {
                    duty = 1; // PID says "move", never send a zero duty
                }
            }
            else
            {
                direction = target > position ? Direction::Opening : Direction::Closing;
                duty = cfg::MOTOR_DRIVE_DUTY_PERCENT;
            }
        }

        if (direction == Direction::Stopped)
        {
            ActuatorInterlock::stopMotor();
        }
        else
        {
            ActuatorInterlock::drive(direction, duty);
        }

        s_activeDirection = direction;
        s_activeIsSeating = seating;
        s_activeDriveDurationMs = driveDurationMs;
        s_positionAtTickStart = position;
        s_tickStartMs = millis();
        s_phase = Phase::Driving;

        s_lastTargetFraction = target;
        s_lastCurrentAmps = 0.0f;
    }

    // Returns true if a mid-travel overcurrent fault was raised.
    bool monitorDrivingPhase()
    {
        if (s_activeDirection == Direction::Stopped)
        {
            return false;
        }

        const float amps = CurrentSensing::readActiveDirectionAmps(s_activeDirection, cfg::CURRENT_SENSE_FILTER_WINDOW_MS);
        s_lastCurrentAmps = amps;

        if (s_activeIsSeating)
        {
            // No current-based fault check during the seating push: it's a
            // bounded, one-shot drive into a stop already confirmed by
            // position, at a duty deliberately high enough to seat firmly.
            return false;
        }

        if (amps > stallThresholdFor(s_activeDirection))
        {
            const bool expectedStop =
                (s_activeDirection == Direction::Opening && isNearOpenStop(s_positionAtTickStart)) ||
                (s_activeDirection == Direction::Closing && isNearClosedStop(s_positionAtTickStart));

            ActuatorInterlock::stopMotor();

            if (expectedStop)
            {
                // Legitimate arrival at a mechanical stop mid-burst, not a
                // fault: resync the dead-reckoning estimate (a no-op in
                // POSITION mode's next sensor read) and let the next tick's
                // decision transition into holding.
                if (s_calibration.mode == ControlMode::Time)
                {
                    s_estimator.resyncTo(s_activeDirection == Direction::Opening ? 1.0f : 0.0f);
                }
                s_phase = Phase::AwaitingTick;
                return false;
            }

            Logger::event(String(F("fault: mid-travel overcurrent, direction=")) + directionSign(s_activeDirection) + F(" amps=") + amps);
            s_phase = Phase::AwaitingTick;
            return true;
        }

        return false;
    }
}

namespace ValvePositionControl
{
    void reset(const HomingCalibration::Result &calibration)
    {
        s_calibration = calibration;
        s_pid.reset();
        s_estimator = DeadReckoningEstimator(calibration.openTimeMs, calibration.closeTimeMs, 1.0f);
        s_phase = Phase::AwaitingTick;
        s_activeDirection = Direction::Stopped;
        s_activeIsSeating = false;
        s_activeDriveDurationMs = cfg::T_DRIVE_MS;
        s_seatedAt = Direction::Stopped;

        Logger::event(String(F("control: reset, mode=")) + (calibration.mode == ControlMode::Position ? F("POSITION") : F("TIME")));
    }

    bool update()
    {
        bool fault = false;

        if (s_phase == Phase::AwaitingTick)
        {
            decideAndStartTick();
        }
        else if (monitorDrivingPhase())
        {
            fault = true;
        }
        else if (s_phase == Phase::Driving && (millis() - s_tickStartMs) >= s_activeDriveDurationMs)
        {
            if (s_activeDirection != Direction::Stopped)
            {
                ActuatorInterlock::stopMotor();

                if (s_calibration.mode == ControlMode::Time)
                {
                    s_estimator.update(directionSign(s_activeDirection), millis() - s_tickStartMs);
                }

                if (s_activeIsSeating)
                {
                    s_seatedAt = s_activeDirection;
                }
            }
            s_phase = Phase::AwaitingTick;
        }

        // Logger::telemetry throttles internally - safe to call every time.
        Logger::telemetry(s_lastTargetFraction, s_positionAtTickStart, directionSign(s_activeDirection),
                           s_lastCurrentAmps, CurrentSensing::readBatteryVolts());

        return fault;
    }
}
