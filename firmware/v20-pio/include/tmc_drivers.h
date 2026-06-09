#pragma once
#include <Arduino.h>
#include "config.h"

namespace tmc {

struct EdgeEvent {
    int  joint;          // 1..3
    const char* label;   // "OT SHUTDOWN", "over-temp WARN", etc.
    uint8_t currentScaling;
    uint32_t millis;
};

using EdgeReporter = void (*)(const EdgeEvent&);

void init();                                       // UART setup + per-driver enable
void setCurrentPct(int jointIdx, int pct);         // 0..100
void enable(int jointIdx);
void disable(int jointIdx);
void disableAll();
void setVelocity(int jointIdx, int32_t vactual);   // signed VACTUAL value

// Periodically (call from main loop): polls 10 Hz, raporteert nieuwe rising edges.
void pollEdgeDetect(EdgeReporter report);

// One-shot snapshot van alle drivers (gebruikt door >TMCSTATUS commando).
struct Snapshot {
    bool ot, otpw, t120, t143, t150, t157;
    bool short_to_ground_a, short_to_ground_b;
    bool low_side_short_a, low_side_short_b;
    bool open_load_a, open_load_b;
    bool standstill, stealth_chop_mode;
    uint8_t current_scaling;
    bool drv_err, uv_cp;
};
Snapshot snapshot(int jointIdx);

// SpreadCycle pinning: schakelt stealthChop automatic-switch uit zodat de
// chopper altijd in spreadCycle (sterker, luidruchtiger) blijft.
// Test-hypothese voor "mini-val + corrigeer" gedrag bij mode-transities.
void forceSpreadCycle(bool on);

} // namespace tmc
