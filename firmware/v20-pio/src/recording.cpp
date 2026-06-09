#include "recording.h"
#include "encoders.h"
#include "tmc_drivers.h"
#include "motion_file.h"
#include <cmath>

namespace recording {

static State st_;

float lowpass(float prev, float now, float alpha) {
    return alpha * now + (1.0f - alpha) * prev;
}

static long armDegToSteps(float deg) {
    return std::lround((deg / 360.0f) * (float)cfg::STEPS_PER_ARM_REV);
}

static bool armAtHome() {
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        float deg = 0.0f;
        if (!encoders::readArmAxis(i, deg)) return false;
        long steps = armDegToSteps(deg);
        if (std::labs(steps) > cfg::REC_HOME_TOLERANCE_STEPS) return false;
    }
    return true;
}

bool start() {
    if (st_.active) return true;
    if (!armAtHome()) return false;
    if (!motion_file::openWrite() || !motion_file::writeHeader()) return false;
    tmc::disableAll();  // freewheel arm zodat user fysiek beweegt
    st_.startMs     = millis();
    st_.lastSampleMs = 0;
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        st_.filtered[i]   = 0.0f;
        st_.filterInit[i] = false;
    }
    st_.active = true;
    return true;
}

void stop() {
    if (!st_.active) return;
    motion_file::flush();
    motion_file::close();
    st_.active = false;
}

void update() {
    if (!st_.active) return;
    uint32_t now = millis();
    if (now - st_.lastSampleMs < cfg::SAMPLE_MS) return;
    st_.lastSampleMs = now;

    motion_file::Sample s;
    s.tMs = now - st_.startMs;
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        float deg = 0.0f;
        if (!encoders::readArmAxis(i, deg)) return;
        float raw = (float)armDegToSteps(deg);
        if (!st_.filterInit[i]) { st_.filtered[i] = raw; st_.filterInit[i] = true; }
        else                    { st_.filtered[i] = lowpass(st_.filtered[i], raw, cfg::REC_FILTER_ALPHA); }
        s.steps[i] = std::lround(st_.filtered[i]);
    }
    motion_file::writeSample(s);

    static uint32_t lastFlush = 0;
    if (now - lastFlush > 2000) { lastFlush = now; motion_file::flush(); }
}

bool active() { return st_.active; }

} // namespace recording
