#pragma once
#include <Arduino.h>
#include "config.h"
#include "pid.h"
#include "soft_endstops.h"

namespace playback {

// Pure: Catmull-Rom Hermite spline tussen p1 en p2, omringd door p0 en p3.
// t in [0,1]. Tangenten worden uit (p2-p0)/2 en (p3-p1)/2 gehaald.
float catmullRom(float p0, float p1, float p2, float p3, float t);

// Pure: lineaire interpolatie tussen twee samples op tijd `now` tussen tA en tB.
float linearInterp(long aSteps, uint32_t aTimeMs,
                   long bSteps, uint32_t bTimeMs,
                   uint32_t nowMs);

// Live state
struct Config {
    pid::Gains            gains [cfg::NUM_JOINTS];
    pid::Config           pidCfg[cfg::NUM_JOINTS];
    soft_endstops::Limits limits[cfg::NUM_JOINTS];
};

bool start(const Config& cfg);
void stop();
void update();  // call vanuit main loop op SAMPLE_MS rate
bool active();

// Reference voor gegeven joint (laatst-berekende ref in motor-stappen)
long lastRefSteps(int jointIdx);
long lastEncSteps(int jointIdx);
float lastErrorDeg(int jointIdx);

} // namespace playback
