#pragma once
#include <Arduino.h>
#include "config.h"
#include "soft_endstops.h"

namespace nvs_storage {

struct HomePose {
    uint16_t motorRaw[cfg::NUM_JOINTS] = {0, 0, 0};
    uint16_t armRaw  [cfg::NUM_JOINTS] = {0, 0, 0};
    bool     present = false;
};

// Home pose
bool loadHome(HomePose& out);
void saveHome(const HomePose& hp);

// Soft-endstop limits per joint
bool loadLimits(soft_endstops::Limits out[cfg::NUM_JOINTS]);
void saveLimits(const soft_endstops::Limits in[cfg::NUM_JOINTS]);
void clearLimits(int jointIdx);

} // namespace nvs_storage
