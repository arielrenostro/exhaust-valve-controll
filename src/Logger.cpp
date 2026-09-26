#include "Logger.h"
#include "config.h"

namespace
{
    unsigned long s_lastTelemetryMs = 0;
    bool s_hasLoggedTelemetry = false;
}

namespace Logger
{
    void begin()
    {
        Serial.begin(115200);
    }

    void event(const String &message)
    {
        Serial.print(F("[EVENT] "));
        Serial.println(message);
    }

    void telemetry(float targetFraction, float positionFraction, int8_t activeDirection, float currentAmps, float batteryVolts)
    {
        const unsigned long now = millis();
        if (s_hasLoggedTelemetry && (now - s_lastTelemetryMs) < cfg::TELEMETRY_LOG_INTERVAL_MS)
        {
            return;
        }
        s_lastTelemetryMs = now;
        s_hasLoggedTelemetry = true;

        Serial.print(F("[TEL] target="));
        Serial.print(targetFraction, 3);
        Serial.print(F(" pos="));
        Serial.print(positionFraction, 3);
        Serial.print(F(" dir="));
        Serial.print(activeDirection);
        Serial.print(F(" I="));
        Serial.print(currentAmps, 2);
        Serial.print(F(" Vbat="));
        Serial.println(batteryVolts, 2);
    }
}
