#pragma once
#include <Arduino.h>

// Non-blocking status LED driven by millis().
// Patterns:
//   OFF           : LED held LOW.
//   DISARMED      : single short flash (50 ms) every 6 s.
//   ARMING_DELAY  : fast blink ~5 Hz (100 ms on / 100 ms off).
//   ARMED         : 3-flash burst, then long pause (car-alarm style).
//   ALARM         : fast double-blink burst, repeating.
class StatusLed {
public:
    enum Mode { OFF, DISARMED, ARMING_DELAY, ARMED, ALARM };

    void begin();
    void setMode(Mode m);
    Mode mode() const { return _mode; }
    void update();                          // tick from main loop

private:
    void writePin(bool on);

    Mode     _mode          = OFF;
    bool     _ledOn         = false;
    uint8_t  _phase         = 0;            // pattern step index (ON/OFF inside one flash)
    uint8_t  _flashIndex    = 0;            // flash counter inside an ARMED burst
    uint32_t _nextChangeMs  = 0;
};
