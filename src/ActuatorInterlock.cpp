#include "ActuatorInterlock.h"
#include "Pins.h"
#include "config.h"

namespace
{
    ActuatorInterlock::Direction s_lastDirection = ActuatorInterlock::Direction::Stopped;

    // Two different components turn off, so two different waits - see
    // config.h for why they're kept separate. Both are applied
    // unconditionally on every transition, regardless of what our own
    // bookkeeping thinks the pin state already is: the hardware guarantee
    // must not depend on that being right.
    void settleAfterBts7960Off()
    {
        delay(cfg::T_BTS7960_TURNOFF_MS);
    }

    void settleAfterReadMosfetOff()
    {
        delayMicroseconds(cfg::T_READ_MOSFET_TURNOFF_US);
    }

    void disableMotorOutputs()
    {
        analogWrite(pins::MOTOR_R_PWM, 0);
        analogWrite(pins::MOTOR_L_PWM, 0);
        digitalWrite(pins::MOTOR_EN, LOW);
    }

    void disableReadCircuit()
    {
        digitalWrite(pins::POSITION_READ_EN, LOW);
    }

    // The Nano's default ADC prescaler (128) gives full accuracy but a slow
    // ~104us conversion. cfg::ADC_PRESCALER trades some of that accuracy for
    // speed - see design.md Decision 3 for why that trade is acceptable here.
    void configureAdcPrescaler()
    {
        ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2));
        switch (cfg::ADC_PRESCALER)
        {
        case 2:
            ADCSRA |= bit(ADPS0);
            break;
        case 4:
            ADCSRA |= bit(ADPS1);
            break;
        case 8:
            ADCSRA |= bit(ADPS1) | bit(ADPS0);
            break;
        case 16:
            ADCSRA |= bit(ADPS2);
            break;
        case 32:
            ADCSRA |= bit(ADPS2) | bit(ADPS0);
            break;
        case 64:
            ADCSRA |= bit(ADPS2) | bit(ADPS1);
            break;
        default:
            ADCSRA |= bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // 128, Arduino core default
            break;
        }
    }
}

namespace ActuatorInterlock
{
    void begin()
    {
        pinMode(pins::MOTOR_R_PWM, OUTPUT);
        pinMode(pins::MOTOR_L_PWM, OUTPUT);
        pinMode(pins::MOTOR_EN, OUTPUT);
        pinMode(pins::POSITION_READ_EN, OUTPUT);
        pinMode(pins::POSITION_SENSE, INPUT);

        analogWrite(pins::MOTOR_R_PWM, 0);
        analogWrite(pins::MOTOR_L_PWM, 0);
        digitalWrite(pins::MOTOR_EN, LOW);
        digitalWrite(pins::POSITION_READ_EN, LOW);

        configureAdcPrescaler();

        s_lastDirection = Direction::Stopped;
    }

    void drive(Direction direction, uint8_t dutyPercent)
    {
        // Never overlap with the read circuit: the D9 MOSFET must have
        // actually finished turning off before D7 can go high.
        disableReadCircuit();
        settleAfterReadMosfetOff();

        // Direction-reversal dead-time: don't flip straight from one
        // nonzero direction to the opposite one - wait for the BTS7960
        // outputs to actually be off first.
        if (s_lastDirection != Direction::Stopped &&
            direction != Direction::Stopped &&
            direction != s_lastDirection)
        {
            analogWrite(pins::MOTOR_R_PWM, 0);
            analogWrite(pins::MOTOR_L_PWM, 0);
            settleAfterBts7960Off();
        }

        const uint8_t pwmValue = (uint8_t)map(constrain(dutyPercent, 0, 100), 0, 100, 0, 255);

        switch (direction)
        {
        case Direction::Opening:
            analogWrite(pins::MOTOR_L_PWM, 0);
            analogWrite(pins::MOTOR_R_PWM, pwmValue);
            break;
        case Direction::Closing:
            analogWrite(pins::MOTOR_R_PWM, 0);
            analogWrite(pins::MOTOR_L_PWM, pwmValue);
            break;
        case Direction::Stopped:
        default:
            analogWrite(pins::MOTOR_R_PWM, 0);
            analogWrite(pins::MOTOR_L_PWM, 0);
            break;
        }

        digitalWrite(pins::MOTOR_EN, direction == Direction::Stopped ? LOW : HIGH);
        s_lastDirection = direction;
    }

    void stopMotor()
    {
        disableMotorOutputs();
        s_lastDirection = Direction::Stopped;
    }

    int readPositionRaw()
    {
        // Always disable and wait, even if the motor is believed to already
        // be off (e.g. a caller just called stopMotor()) - the settle
        // guarantee must hold independently of that bookkeeping.
        disableMotorOutputs();
        settleAfterBts7960Off();

        digitalWrite(pins::POSITION_READ_EN, HIGH);
        const int raw = analogRead(pins::POSITION_SENSE);
        digitalWrite(pins::POSITION_READ_EN, LOW);

        return raw;
    }
}
