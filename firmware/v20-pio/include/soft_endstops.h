#pragma once
#include <cmath>

namespace soft_endstops {

struct Limits {
    float minDeg = NAN;   // NaN = unset
    float maxDeg = NAN;
};

struct ClampResult {
    float refDeg;
    bool  hitMin;
    bool  hitMax;
};

// Pure: clamp ref to current's position if outside limits (= hold position).
// Returnt aangepaste ref + welke kant geraakt is.
ClampResult clamp(float refDeg, float currentDeg, const Limits& lim);

// Validatie van limits
inline bool isSet(const Limits& lim) {
    return !std::isnan(lim.minDeg) || !std::isnan(lim.maxDeg);
}

} // namespace soft_endstops
