#include "tmc_drivers.h"
#include <TMC2209.h>

namespace tmc {

static TMC2209 drv_[cfg::NUM_JOINTS];
static bool    forceSpread_ = false;

struct FlagState {
    bool ot, otpw, t157, t150, t143, t120;
    bool s2g_a, s2g_b, ls_a, ls_b, ol_a, ol_b;
    bool drv_err, uv;
};
static FlagState last_[cfg::NUM_JOINTS] = {};

void init() {
    Serial1.begin(115200, SERIAL_8N1, cfg::TMC_RX_PIN, cfg::TMC_TX_PIN);
    pinMode(cfg::TMC_EN_PIN, OUTPUT);
    digitalWrite(cfg::TMC_EN_PIN, LOW);  // enable per-driver via UART, EN op LOW = global enable

    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        drv_[i].setup(Serial1, 115200, (TMC2209::SerialAddress)cfg::TMC_ADDRS[i]);
        drv_[i].setRunCurrent(cfg::TMC_DEFAULT_CURRENT_PCT);
        drv_[i].setHoldCurrent(cfg::TMC_DEFAULT_CURRENT_PCT);
        drv_[i].setMicrostepsPerStep(cfg::MICROSTEPS);
        if (forceSpread_) drv_[i].disableStealthChop();
        else              drv_[i].enableStealthChop();
        drv_[i].enable();
    }
}

void setCurrentPct(int jointIdx, int pct) {
    if (jointIdx < 0 || jointIdx >= cfg::NUM_JOINTS) return;
    if (pct < 0) pct = 0; if (pct > 100) pct = 100;
    drv_[jointIdx].setRunCurrent((uint8_t)pct);
    drv_[jointIdx].setHoldCurrent((uint8_t)pct);
}

void enable(int jointIdx)   { if (jointIdx >= 0 && jointIdx < cfg::NUM_JOINTS) drv_[jointIdx].enable(); }
void disable(int jointIdx)  { if (jointIdx >= 0 && jointIdx < cfg::NUM_JOINTS) drv_[jointIdx].disable(); }
void disableAll()           { for (int i = 0; i < cfg::NUM_JOINTS; i++) drv_[i].disable(); }

void setVelocity(int jointIdx, int32_t vactual) {
    if (jointIdx < 0 || jointIdx >= cfg::NUM_JOINTS) return;
    drv_[jointIdx].moveAtVelocity(vactual);
}

void forceSpreadCycle(bool on) {
    forceSpread_ = on;
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        if (on) drv_[i].disableStealthChop();
        else    drv_[i].enableStealthChop();
    }
}

#define EDGE(flagNow, prev_field, lbl) \
    do { if ((flagNow) && !(prev.prev_field)) { \
        if (report) { EdgeEvent e{i+1, (lbl), (uint8_t)s.current_scaling, now}; report(e); } \
    } prev.prev_field = (flagNow); } while (0)

void pollEdgeDetect(EdgeReporter report) {
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < cfg::TMC_STATUS_POLL_MS) return;
    lastMs = now;

    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        TMC2209::Status       s  = drv_[i].getStatus();
        TMC2209::GlobalStatus gs = drv_[i].getGlobalStatus();
        FlagState& prev = last_[i];
        EDGE(s.over_temperature_shutdown, ot,    "OT SHUTDOWN (>157C)");
        EDGE(s.over_temperature_warning,  otpw,  "over-temp WARN (>120C)");
        EDGE(s.over_temperature_157c,     t157,  "temp >=157C");
        EDGE(s.over_temperature_150c,     t150,  "temp >=150C");
        EDGE(s.over_temperature_143c,     t143,  "temp >=143C");
        EDGE(s.over_temperature_120c,     t120,  "temp >=120C");
        EDGE(s.short_to_ground_a,         s2g_a, "short-to-GND coil A");
        EDGE(s.short_to_ground_b,         s2g_b, "short-to-GND coil B");
        EDGE(s.low_side_short_a,          ls_a,  "low-side short coil A");
        EDGE(s.low_side_short_b,          ls_b,  "low-side short coil B");
        EDGE(s.open_load_a,               ol_a,  "open-load coil A");
        EDGE(s.open_load_b,               ol_b,  "open-load coil B");
        EDGE(gs.drv_err,                  drv_err, "GLOBAL drv_err");
        EDGE(gs.uv_cp,                    uv,    "undervoltage charge-pump");
    }
}
#undef EDGE

Snapshot snapshot(int jointIdx) {
    Snapshot snap{};
    if (jointIdx < 0 || jointIdx >= cfg::NUM_JOINTS) return snap;
    auto s  = drv_[jointIdx].getStatus();
    auto gs = drv_[jointIdx].getGlobalStatus();
    snap.ot                = s.over_temperature_shutdown;
    snap.otpw              = s.over_temperature_warning;
    snap.t120              = s.over_temperature_120c;
    snap.t143              = s.over_temperature_143c;
    snap.t150              = s.over_temperature_150c;
    snap.t157              = s.over_temperature_157c;
    snap.short_to_ground_a = s.short_to_ground_a;
    snap.short_to_ground_b = s.short_to_ground_b;
    snap.low_side_short_a  = s.low_side_short_a;
    snap.low_side_short_b  = s.low_side_short_b;
    snap.open_load_a       = s.open_load_a;
    snap.open_load_b       = s.open_load_b;
    snap.standstill        = s.standstill;
    snap.stealth_chop_mode = s.stealth_chop_mode;
    snap.current_scaling   = s.current_scaling;
    snap.drv_err           = gs.drv_err;
    snap.uv_cp             = gs.uv_cp;
    return snap;
}

} // namespace tmc
