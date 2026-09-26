#include "PwmTargetReader.h"
#include <TimerOne.h>

namespace
{
    volatile uint8_t s_pin = 0;
    volatile uint8_t s_lastState = LOW;
    volatile unsigned long s_lastRiseMicros = 0;
    volatile unsigned long s_periodMicros = 0;
    volatile unsigned long s_highMicros = 0;
    volatile unsigned long s_lastEdgeMicros = 0;
    volatile float s_dutyPercent = 0.0f;

    unsigned long s_timeoutMicros = 0;
    volatile bool s_valid = false;

    void onEdge()
    {
        const unsigned long now = micros();
        const uint8_t state = digitalRead(s_pin);

        if (state != s_lastState)
        {
            s_lastState = state;
            s_lastEdgeMicros = now;

            if (state == HIGH)
            {
                // Rising edge closes the previous full period.
                s_periodMicros = now - s_lastRiseMicros;
                s_lastRiseMicros = now;
            }
            else
            {
                // Falling edge closes this period's HIGH time.
                s_highMicros = now - s_lastRiseMicros;
                if (s_periodMicros > 0)
                {
                    s_dutyPercent = (100.0f * (float)s_highMicros) / (float)s_periodMicros;
                }
            }
        }
    }

    void checkWatchdog()
    {
        const unsigned long now = micros();
        const unsigned long sinceLastEdge = now - s_lastEdgeMicros;
        s_valid = s_lastEdgeMicros != 0 && sinceLastEdge <= s_timeoutMicros;
    }
}

namespace PwmTargetReader
{
    void begin(uint8_t pin, unsigned long timeoutMs)
    {
        s_pin = pin;
        s_timeoutMicros = timeoutMs * 1000UL;

        pinMode(s_pin, INPUT);
        s_lastState = digitalRead(s_pin);
        s_lastRiseMicros = micros();
        s_lastEdgeMicros = 0; // no edge observed yet -> invalid until the first one arrives
        s_valid = false;

        attachInterrupt(digitalPinToInterrupt(s_pin), onEdge, CHANGE);

        // Watchdog tick runs a few times per timeout window so signal loss is
        // detected promptly without needing a very short timer period.
        unsigned long watchdogPeriodUs = s_timeoutMicros / 4;
        if (watchdogPeriodUs < 1000UL)
        {
            watchdogPeriodUs = 1000UL;
        }
        Timer1.initialize(watchdogPeriodUs);
        Timer1.attachInterrupt(checkWatchdog);
    }

    float getDutyPercent()
    {
        noInterrupts();
        const float duty = s_dutyPercent;
        interrupts();
        return duty;
    }

    bool isValid()
    {
        return s_valid;
    }
}
