#pragma once
#include <Arduino.h>

// Phase 5 placeholder.
class Button {
public:
    enum Event { NONE, SHORT_PRESS, LONG_PRESS };
    void  begin();
    Event poll();   // returns event since last call
};
