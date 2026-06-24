#pragma once
#include <Arduino.h>
#include "config.h"

// MOSFET-driven siren on PIN_SIREN.
//   on()            : steady ON (logged once per state change)
//   off()           : steady OFF (logged once per state change; cancels any
//                     in-flight pattern or confirmation beep)
//   alarmPattern()  : non-blocking ALARM pattern. In DEBUG_SIREN_SAFE_MODE
//                     this is a low-duty-cycle 200/800 pulse train.
//   confirmArm()    : non-blocking N short beeps (arm acknowledgement)
//   confirmDisarm() : non-blocking N short beeps (disarm acknowledgement)
//   update()        : tick from main loop; advances the active pattern.
class Siren {
public:
    enum Mode { IDLE, STEADY_ON, ALARM_PATTERN, CONFIRM_BEEP };

    void begin();
    void on();
    void off();
    void alarmPattern();

    void confirmArm();
    void confirmDisarm();
    bool isConfirmationActive() const { return _mode == CONFIRM_BEEP; }

    void update();

    bool isOn() const { return _on; }
    Mode mode() const { return _mode; }
    bool isAlarmSirenActive() const {
        return _mode == ALARM_PATTERN && !_alarmContinuousFinished;
    }

private:
    void writePin(bool on);
    void startConfirmBeeps(uint8_t count);

    Mode     _mode             = IDLE;
    bool     _on               = false;
    uint32_t _nextChangeMs     = 0;

    // CONFIRM_BEEP sequence state.
    uint8_t  _confirmBeepsLeft = 0;
    bool     _confirmInOnPhase = false;

    // ALARM_PATTERN continuous-mode state. _alarmStartMs is set on entry;
    // _alarmContinuousFinished latches true after SIREN_ALARM_CONTINUOUS_MS
    // so the timeout log line prints exactly once and the pin stays off
    // (unless the optional reminder pulse train is enabled).
    uint32_t _alarmStartMs            = 0;
    bool     _alarmContinuousFinished = false;
};
