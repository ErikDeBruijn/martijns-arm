#pragma once
#include <Arduino.h>
#include <FastLED.h>

namespace led {

// 1-op-1 met v19 LedAnim enum (volgorde + namen)
enum Anim { OFF, SOLID, PULSE_SLOW, PULSE_FAST, BLINK_1HZ, BLINK_5HZ, PROGRESS, FLASH3 };

struct State {
    Anim     anim       = OFF;
    CRGB     color      = CRGB::Black;
    float    progress   = 0.0f;
    uint32_t lastMs     = 0;
    uint8_t  flashCount = 0;
    bool     flashOn    = false;
    bool     flashDone  = false;
};

void init();
void set(Anim anim, CRGB color, float progress = 0.0f);
void update();

} // namespace led
