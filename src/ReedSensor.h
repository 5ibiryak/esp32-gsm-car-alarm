#pragma once
#include <Arduino.h>

// Door reed switch (normally-closed when the magnet is near the contact).
//   closed (magnet present) -> pin reads LOW
//   open   (magnet removed) -> pin reads HIGH via INPUT_PULLUP
class ReedSensor {
public:
    void begin();
    void update();                          // tick from main loop

    bool isOpen()  const { return _open; }
    bool changed() const { return _justChanged; }

private:
    bool     _open           = false;       // current debounced state
    bool     _justChanged    = false;       // true for one update() after a debounced edge
    bool     _rawOpen        = false;       // last raw read
    uint32_t _lastRawEdgeMs  = 0;
    bool     _initialized    = false;
};
