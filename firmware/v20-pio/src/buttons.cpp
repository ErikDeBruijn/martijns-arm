#include "buttons.h"
#include "config.h"

namespace buttons {

namespace {
    struct EdgeState {
        uint32_t lastChangeMs = 0;
        uint8_t  lastState    = HIGH;
    };
    EdgeState edgeRec_, edgePlay_;

    struct LongPress {
        uint32_t t0      = 0;
        bool     armed   = false;
        bool     fired   = false;

        bool update(int pin, uint32_t holdMs) {
            if (digitalRead(pin) == LOW) {
                if (!armed) { armed = true; fired = false; t0 = millis(); }
                if (!fired && (millis() - t0 >= holdMs)) { fired = true; return true; }
            } else { armed = false; fired = false; }
            return false;
        }
    };
    LongPress lpPlay_, lpDel_;

    bool fellEdge(int pin, EdgeState& st) {
        uint8_t  s   = (uint8_t)digitalRead(pin);
        uint32_t now = millis();
        if (s != st.lastState && (now - st.lastChangeMs) > 30) {
            st.lastChangeMs = now;
            st.lastState    = s;
            if (s == LOW) return true;
        }
        return false;
    }
}

void init() {
    pinMode(cfg::BTN_REC_PIN,  INPUT_PULLUP);
    pinMode(cfg::BTN_PLAY_PIN, INPUT_PULLUP);
    pinMode(cfg::BTN_DEL_PIN,  INPUT_PULLUP);
}

Event poll() {
    // Lang-press eerst (consumeren rising edge zodat short-press niet onterecht triggert)
    if (lpDel_.update(cfg::BTN_DEL_PIN, cfg::DEL_HOLD_MS))    return Event::DEL_LONG;
    if (lpPlay_.update(cfg::BTN_PLAY_PIN, cfg::BTN_HOLD_MS))  return Event::PLAY_LONG;

    // Short-press = falling edge (debounced)
    if (fellEdge(cfg::BTN_REC_PIN,  edgeRec_))                return Event::REC_SHORT;
    // Short-press op PLAY alleen als niet bezig met long-press
    if (fellEdge(cfg::BTN_PLAY_PIN, edgePlay_))               return Event::PLAY_SHORT;

    return Event::NONE;
}

} // namespace buttons
