#include "nvs_storage.h"
#include <Preferences.h>
#include <cmath>

namespace nvs_storage {

static Preferences prefs;

bool loadHome(HomePose& out) {
    prefs.begin(cfg::NVS_NS, true);
    bool present = prefs.getBool("home_present", false);
    if (present) {
        for (int i = 0; i < cfg::NUM_JOINTS; i++) {
            char k[16];
            snprintf(k, sizeof(k), "hm_%d", i); out.motorRaw[i] = prefs.getUShort(k, 0);
            snprintf(k, sizeof(k), "ha_%d", i); out.armRaw  [i] = prefs.getUShort(k, 0);
        }
    }
    out.present = present;
    prefs.end();
    return present;
}

void saveHome(const HomePose& hp) {
    prefs.begin(cfg::NVS_NS, false);
    prefs.putBool("home_present", true);
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        char k[16];
        snprintf(k, sizeof(k), "hm_%d", i); prefs.putUShort(k, hp.motorRaw[i]);
        snprintf(k, sizeof(k), "ha_%d", i); prefs.putUShort(k, hp.armRaw  [i]);
    }
    prefs.end();
}

bool loadLimits(soft_endstops::Limits out[cfg::NUM_JOINTS]) {
    prefs.begin(cfg::NVS_NS, true);
    bool any = false;
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        char k[16];
        snprintf(k, sizeof(k), "lmn%d", i); out[i].minDeg = prefs.getFloat(k, NAN);
        snprintf(k, sizeof(k), "lmx%d", i); out[i].maxDeg = prefs.getFloat(k, NAN);
        if (soft_endstops::isSet(out[i])) any = true;
    }
    prefs.end();
    return any;
}

void saveLimits(const soft_endstops::Limits in[cfg::NUM_JOINTS]) {
    prefs.begin(cfg::NVS_NS, false);
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        char k[16];
        snprintf(k, sizeof(k), "lmn%d", i); prefs.putFloat(k, in[i].minDeg);
        snprintf(k, sizeof(k), "lmx%d", i); prefs.putFloat(k, in[i].maxDeg);
    }
    prefs.end();
}

void clearLimits(int jointIdx) {
    if (jointIdx < 0 || jointIdx >= cfg::NUM_JOINTS) return;
    prefs.begin(cfg::NVS_NS, false);
    char k[16];
    snprintf(k, sizeof(k), "lmn%d", jointIdx); prefs.putFloat(k, NAN);
    snprintf(k, sizeof(k), "lmx%d", jointIdx); prefs.putFloat(k, NAN);
    prefs.end();
}

} // namespace nvs_storage
