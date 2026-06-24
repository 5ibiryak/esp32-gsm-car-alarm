#include "Siren.h"
#include "config.h"
#include "RuntimeConfig.h"

void Siren::begin() {
    pinMode(PIN_SIREN, OUTPUT);
    digitalWrite(PIN_SIREN, LOW);
    _mode             = IDLE;
    _on               = false;
    _confirmBeepsLeft = 0;
    _confirmInOnPhase = false;

#if DEBUG_SIREN
    Serial.print  (F("[SIREN] begin() — pin GPIO"));
    Serial.println(PIN_SIREN);
#if DEBUG_SIREN_SAFE_MODE
    Serial.print  (F("[SIREN] safe mode: "));
    Serial.print  (SIREN_SAFE_ON_MS);
    Serial.print  (F(" ms ON / "));
    Serial.print  (SIREN_SAFE_OFF_MS);
    Serial.println(F(" ms OFF"));
#endif
#endif
}

void Siren::writePin(bool on) {
    if (on == _on) return;
    _on = on;
    digitalWrite(PIN_SIREN, on ? HIGH : LOW);
#if DEBUG_SIREN
    Serial.println(on ? F("[SIREN] ON") : F("[SIREN] OFF"));
#endif
}

void Siren::on() {
    _mode                     = STEADY_ON;
    _confirmBeepsLeft         = 0;
    _confirmInOnPhase         = false;
    _alarmStartMs             = 0;
    _alarmContinuousFinished  = false;
    writePin(true);
}

void Siren::off() {
    _mode                     = IDLE;
    _confirmBeepsLeft         = 0;
    _confirmInOnPhase         = false;
    _alarmStartMs             = 0;
    _alarmContinuousFinished  = false;
    writePin(false);
}

void Siren::alarmPattern() {
    if (_mode == ALARM_PATTERN) return;
    _mode             = ALARM_PATTERN;
    _confirmBeepsLeft = 0;
    _confirmInOnPhase = false;

#if SIREN_ALARM_MODE_CONTINUOUS
    _alarmStartMs            = millis();
    _alarmContinuousFinished = false;
    writePin(true);
    const uint32_t contMs = runtimeConfig.sirenAlarmContinuousMs();
#if DEBUG_SIREN
    Serial.print  (F("[SIREN] ALARM continuous ON for "));
    Serial.print  (contMs / 1000UL);
    Serial.println(F("s"));
#endif
    // _nextChangeMs is unused in the continuous phase; update() compares
    // millis() - _alarmStartMs against the runtime continuous window
    // directly.
    _nextChangeMs = _alarmStartMs + contMs;
#else
    writePin(true);
#if DEBUG_SIREN_SAFE_MODE
    _nextChangeMs = millis() + SIREN_SAFE_ON_MS;
#else
    _nextChangeMs = millis() + 0x7FFFFFFFUL;
#endif
#endif
}

void Siren::startConfirmBeeps(uint8_t count) {
    if (count == 0) {
        off();
        return;
    }
    _mode              = CONFIRM_BEEP;
    _confirmBeepsLeft  = count;
    _confirmInOnPhase  = false;     // next update() will start the first ON pulse
    _nextChangeMs      = millis();  // immediate trigger
    writePin(false);
}

void Siren::confirmArm() {
#if DEBUG_SIREN
    Serial.print  (F("[SIREN] confirm arm: "));
    Serial.print  (SIREN_ARM_CONFIRM_BEEPS);
    Serial.println((SIREN_ARM_CONFIRM_BEEPS == 1) ? F(" beep") : F(" beeps"));
#endif
    startConfirmBeeps(SIREN_ARM_CONFIRM_BEEPS);
}

void Siren::confirmDisarm() {
#if DEBUG_SIREN
    Serial.print  (F("[SIREN] confirm disarm: "));
    Serial.print  (SIREN_DISARM_CONFIRM_BEEPS);
    Serial.println((SIREN_DISARM_CONFIRM_BEEPS == 1) ? F(" beep") : F(" beeps"));
#endif
    startConfirmBeeps(SIREN_DISARM_CONFIRM_BEEPS);
}

void Siren::update() {
    const uint32_t now = millis();

    if (_mode == CONFIRM_BEEP) {
        if ((int32_t)(now - _nextChangeMs) < 0) return;

        if (_confirmInOnPhase) {
            // End the current ON pulse.
            writePin(false);
            _confirmInOnPhase = false;
            if (_confirmBeepsLeft > 0) --_confirmBeepsLeft;
            if (_confirmBeepsLeft == 0) {
                _mode = IDLE;
                return;
            }
            _nextChangeMs = now + SIREN_CONFIRM_GAP_MS;
        } else {
            // Start a new ON pulse.
            writePin(true);
            _confirmInOnPhase = true;
            _nextChangeMs = now + SIREN_CONFIRM_BEEP_MS;
        }
        return;
    }

    if (_mode != ALARM_PATTERN) return;

#if SIREN_ALARM_MODE_CONTINUOUS
    // Continuous-ON window. The pin was raised in alarmPattern(); nothing
    // to do until the runtime-configurable window elapses. The window
    // length is sampled at entry into the ALARM state and stored on
    // _alarmStartMs + window — runtime changes only affect future alarms.
    if (!_alarmContinuousFinished) {
        const uint32_t contMs = runtimeConfig.sirenAlarmContinuousMs();
        if ((int32_t)(now - (_alarmStartMs + contMs)) < 0) return;

        writePin(false);
        _alarmContinuousFinished = true;
#if DEBUG_SIREN
        Serial.println(F("[SIREN] ALARM continuous timeout, siren OFF"));
#endif
        if (runtimeConfig.sirenAfterTimeoutReminderEnabled()) {
            _nextChangeMs = now + runtimeConfig.sirenReminderOffMs();
        }
        return;
    }

    if (runtimeConfig.sirenAfterTimeoutReminderEnabled()) {
        if ((int32_t)(now - _nextChangeMs) < 0) return;
        if (_on) {
            writePin(false);
            _nextChangeMs = now + runtimeConfig.sirenReminderOffMs();
        } else {
            writePin(true);
            _nextChangeMs = now + runtimeConfig.sirenReminderOnMs();
        }
    }
#else  // !SIREN_ALARM_MODE_CONTINUOUS
#if DEBUG_SIREN_SAFE_MODE
    if ((int32_t)(now - _nextChangeMs) < 0) return;
    if (_on) {
        writePin(false);
        _nextChangeMs = now + SIREN_SAFE_OFF_MS;
    } else {
        writePin(true);
        _nextChangeMs = now + SIREN_SAFE_ON_MS;
    }
#endif
#endif
}
