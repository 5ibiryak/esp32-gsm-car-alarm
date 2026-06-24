#include "StatusLed.h"
#include "config.h"

void StatusLed::begin() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    _mode         = OFF;
    _ledOn        = false;
    _phase        = 0;
    _flashIndex   = 0;
    _nextChangeMs = millis();

#if DEBUG_LED
    Serial.print  (F("[LED] begin() — pin GPIO"));
    Serial.println(PIN_STATUS_LED);
    Serial.print  (F("[LED] ARMED pattern: "));
    Serial.print  (LED_ARMED_FLASH_COUNT);
    Serial.print  (F(" flashes, "));
    Serial.print  (LED_ARMED_FLASH_ON_MS);
    Serial.print  (F("ms ON, "));
    Serial.print  (LED_ARMED_FLASH_OFF_MS);
    Serial.print  (F("ms gap, "));
    Serial.print  (LED_ARMED_PAUSE_MS);
    Serial.println(F("ms pause"));
#endif
}

void StatusLed::writePin(bool on) {
    _ledOn = on;
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void StatusLed::setMode(Mode m) {
    if (m == _mode) return;
    _mode         = m;
    _phase        = 0;
    _flashIndex   = 0;
    _nextChangeMs = millis();
    // Force the next update() to advance into the new pattern immediately
    // by pretending the previous step has already elapsed.
    writePin(false);
}

void StatusLed::update() {
    const uint32_t now = millis();
    if ((int32_t)(now - _nextChangeMs) < 0) return;

    switch (_mode) {
        case OFF:
            writePin(false);
            _nextChangeMs = now + 1000;             // idle tick — nothing to do
            break;

        case DISARMED:
            // Single short flash, long quiet gap (~6 s period).
            if (_phase == 0) {
                writePin(true);
                _nextChangeMs = now + LED_DISARMED_FLASH_MS;
                _phase = 1;
            } else {
                writePin(false);
                _nextChangeMs = now + (LED_DISARMED_PERIOD_MS - LED_DISARMED_FLASH_MS);
                _phase = 0;
            }
            break;

        case ARMING_DELAY:
            // 5 Hz square wave: 100 ms on / 100 ms off.
            writePin(!_ledOn);
            _nextChangeMs = now + LED_ARMING_HALF_PERIOD_MS;
            break;

        case ARMED:
            // "Car alarm style": LED_ARMED_FLASH_COUNT short flashes, then a
            // long quiet pause, repeat. Each flash = ON for FLASH_ON_MS,
            // OFF for FLASH_OFF_MS (or PAUSE_MS after the last flash).
            if (_phase == 0) {                              // start of flash -> ON
                writePin(true);
                _nextChangeMs = now + LED_ARMED_FLASH_ON_MS;
                _phase = 1;
            } else {                                         // end of flash -> OFF
                writePin(false);
                _flashIndex++;
                if (_flashIndex < LED_ARMED_FLASH_COUNT) {
                    _nextChangeMs = now + LED_ARMED_FLASH_OFF_MS;
                } else {
                    _nextChangeMs = now + LED_ARMED_PAUSE_MS;
                    _flashIndex   = 0;
                }
                _phase = 0;
            }
            break;

        case ALARM:
            // Four-step double-blink: ON, OFF, ON, long OFF.
            switch (_phase) {
                case 0: writePin(true);  _nextChangeMs = now + LED_ALARM_ON_MS;    break;
                case 1: writePin(false); _nextChangeMs = now + LED_ALARM_OFF_MS;   break;
                case 2: writePin(true);  _nextChangeMs = now + LED_ALARM_ON_MS;    break;
                case 3: writePin(false); _nextChangeMs = now + LED_ALARM_PAUSE_MS; break;
                default: break;
            }
            _phase = (uint8_t)((_phase + 1) & 0x03);
            break;
    }
}
