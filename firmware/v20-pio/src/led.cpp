#include "led.h"
#include "config.h"

namespace led {

static State        state_;
static CRGB         leds_[cfg::LED_COUNT];

void init() {
    FastLED.addLeds<WS2812, cfg::LED_PIN, GRB>(leds_, cfg::LED_COUNT);
    FastLED.setBrightness(cfg::LED_BRIGHTNESS);
    set(OFF, CRGB::Black);
}

void set(Anim anim, CRGB color, float progress) {
    state_.anim       = anim;
    state_.color      = color;
    state_.progress   = progress;
    state_.lastMs     = millis();
    state_.flashCount = 0;
    state_.flashOn    = false;
    state_.flashDone  = false;
}

void update() {
    // 1-op-1 met v19 ledUpdate()
    uint32_t dt = millis() - state_.lastMs;
    uint8_t  br = 0;
    switch (state_.anim) {
        case OFF:   leds_[0] = CRGB::Black; break;
        case SOLID: leds_[0] = state_.color; break;
        case PULSE_SLOW: {
            float s = sinf(((float)(dt % 2000) / 2000.0f) * 2.0f * PI);
            leds_[0] = state_.color; leds_[0].nscale8((uint8_t)(s*s*200.0f+10.0f)); break;
        }
        case PULSE_FAST: {
            float s = sinf(((float)(dt % 500) / 500.0f) * 2.0f * PI);
            leds_[0] = state_.color; leds_[0].nscale8((uint8_t)(s*s*200.0f+20.0f)); break;
        }
        case BLINK_1HZ:
            leds_[0] = ((dt % 1000) < 500) ? state_.color : CRGB::Black; break;
        case BLINK_5HZ:
            leds_[0] = ((dt %  200) < 100) ? state_.color : CRGB::Black; break;
        case PROGRESS: {
            br = (uint8_t)(state_.progress * 220.0f + 10.0f);
            leds_[0] = state_.color; leds_[0].nscale8(br); break;
        }
        case FLASH3: {
            if (state_.flashDone) { leds_[0] = state_.color; break; }
            bool on = ((dt % 240) < 120);
            if (on != state_.flashOn) {
                state_.flashOn = on;
                if (!on) state_.flashCount++;
            }
            if (state_.flashCount >= 3) { state_.flashDone = true; leds_[0] = state_.color; }
            else leds_[0] = on ? state_.color : CRGB::Black;
            break;
        }
    }
    FastLED.show();
}

} // namespace led
