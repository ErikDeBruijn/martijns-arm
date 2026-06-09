#pragma once
#include <Arduino.h>
#include "config.h"

namespace recording {

// Pure: low-pass filter step. alpha=1.0 → geen filter, alpha=0 → infinite hold.
float lowpass(float prev, float now, float alpha);

struct State {
    uint32_t startMs   = 0;
    uint32_t lastSampleMs = 0;
    float    filtered[cfg::NUM_JOINTS] = {0};
    bool     filterInit[cfg::NUM_JOINTS] = {false, false, false};
    bool     active   = false;
};

// Start: check home-tolerance, open motion file, write header, disable motoren.
// Returnt false als arm niet bij home is (per joint check tegen REC_HOME_TOLERANCE_STEPS).
bool start();
void stop();

// Poll vanuit main loop. No-op als !active of binnen SAMPLE_MS sinds laatste call.
void update();

bool active();

} // namespace recording
