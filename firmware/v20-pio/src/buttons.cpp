#include "buttons.h"
#include "config.h"

namespace buttons {

void init() {
    pinMode(cfg::BTN_REC_PIN,  INPUT_PULLUP);
    pinMode(cfg::BTN_PLAY_PIN, INPUT_PULLUP);
    pinMode(cfg::BTN_DEL_PIN,  INPUT_PULLUP);
}

// 1-op-1 met v19 fellEdge
bool fellEdge(int pin) {
    static uint32_t lastChange[64] = {0};
    static uint8_t  lastState[64]  = {1};
    if (pin < 0 || pin >= 64) return false;
    uint8_t  s   = (uint8_t)digitalRead(pin);
    uint32_t now = millis();
    if (s != lastState[pin] && (now - lastChange[pin]) > 30) {
        lastChange[pin] = now; lastState[pin] = s;
        if (s == LOW) return true;
    }
    return false;
}

// 1-op-1 met v19 LongPress::update
bool LongPress::update(int pin, uint32_t holdMs) {
    if (digitalRead(pin) == LOW) {
        if (!armed) { armed = true; fired = false; t0 = millis(); }
        if (!fired && (millis() - t0 >= holdMs)) { fired = true; return true; }
    } else { armed = false; fired = false; }
    return false;
}

float LongPress::progress(uint32_t holdMs) {
    if (!armed) return 0.0f;
    float p = (float)(millis() - t0) / (float)holdMs;
    return (p > 1.0f) ? 1.0f : p;
}

} // namespace buttons
