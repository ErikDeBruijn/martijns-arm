#include "led.h"
#include "config.h"

namespace led {

static State        state_;
static CRGB         leds_[cfg::LED_COUNT];
static uint32_t     animStartMs_ = 0;

bool isOnAt(Anim anim, uint32_t nowMs) {
    switch (anim) {
        case OFF:       return false;
        case SOLID:     return true;
        case BLINK_1HZ: return (nowMs % 1000) < 500;
        case BLINK_5HZ: return (nowMs %  200) < 100;
        case PULSE:     return (nowMs %  800) < 400;  // gelijkmatig in/uit
    }
    return false;
}

void init() {
    FastLED.addLeds<WS2812B, cfg::LED_PIN, GRB>(leds_, cfg::LED_COUNT);
    FastLED.setBrightness(64);
    set(OFF, CRGB::Black);
}

void set(Anim anim, CRGB color) {
    state_.anim   = anim;
    state_.color  = color;
    animStartMs_  = millis();
    update();
}

void update() {
    bool on = isOnAt(state_.anim, millis() - animStartMs_);
    leds_[0] = on ? state_.color : CRGB::Black;
    FastLED.show();
}

} // namespace led
