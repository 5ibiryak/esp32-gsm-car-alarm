#include "ReedSensor.h"
#include "config.h"

void ReedSensor::begin() {
    pinMode(PIN_REED, INPUT_PULLUP);

    // Sample once so the first update() doesn't generate a spurious edge.
    _rawOpen        = (digitalRead(PIN_REED) == HIGH);
    _open           = _rawOpen;
    _lastRawEdgeMs  = millis();
    _initialized    = true;

#if DEBUG_REED
    Serial.print  (F("[REED] begin() — pin GPIO"));
    Serial.print  (PIN_REED);
    Serial.print  (F(" INPUT_PULLUP, debounce="));
    Serial.print  (REED_DEBOUNCE_MS);
    Serial.println(F(" ms"));
    Serial.println(_open ? F("[REED] OPEN") : F("[REED] CLOSED"));
#endif
}

void ReedSensor::update() {
    _justChanged = false;
    if (!_initialized) return;

    const bool raw = (digitalRead(PIN_REED) == HIGH);   // HIGH = open, LOW = closed
    const uint32_t now = millis();

    if (raw != _rawOpen) {
        _rawOpen       = raw;
        _lastRawEdgeMs = now;
        return;                  // wait until the raw level stabilises
    }

    if (raw != _open && (now - _lastRawEdgeMs) >= REED_DEBOUNCE_MS) {
        _open        = raw;
        _justChanged = true;
#if DEBUG_REED
        Serial.println(_open ? F("[REED] OPEN") : F("[REED] CLOSED"));
#endif
    }
}
