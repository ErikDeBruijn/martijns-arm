#pragma once
#include <Arduino.h>

// 1-op-1 utilities uit v19: fellEdge + LongPress
namespace buttons {

void init();

// Debounced falling-edge detect (30ms). Voor short-press.
bool fellEdge(int pin);

// Long-press detector struct (1-op-1 met v19 LongPress).
struct LongPress {
    uint32_t t0 = 0;
    bool     armed = false;
    bool     fired = false;

    bool update(int pin, uint32_t holdMs);
    float progress(uint32_t holdMs);
    bool  isHolding() { return armed && !fired; }
};

} // namespace buttons
