#pragma once
#include <Arduino.h>
#include <FastLED.h>

namespace led {

enum Anim { OFF, SOLID, BLINK_1HZ, BLINK_5HZ, PULSE };

struct State {
    Anim    anim     = OFF;
    CRGB    color    = CRGB::Black;
    uint8_t progress = 0;  // 0..255 voor pulse
};

void init();
void set(Anim anim, CRGB color);
void update();                       // call in main loop, niet-blocking

// Pure functie: bepaal of LED op moment `nowMs` aan moet zijn voor gegeven anim.
// Wordt gebruikt door update() én is unit-testbaar.
bool isOnAt(Anim anim, uint32_t nowMs);

} // namespace led
