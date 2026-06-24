#include "Button.h"
#include "config.h"

void Button::begin() {
#if DEBUG_BUTTON
    Serial.println(F("[BTN] begin() — stub (Phase 5 will configure INPUT_PULLUP + debounce)"));
#endif
}

Button::Event Button::poll() { return NONE; }
