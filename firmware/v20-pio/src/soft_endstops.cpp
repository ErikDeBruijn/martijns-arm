#include "soft_endstops.h"

namespace soft_endstops {

ClampResult clamp(float refDeg, float currentDeg, const Limits& lim) {
    ClampResult r{refDeg, false, false};
    // Normaliseer: accepteer min/max in beide volgordes (sensor-richting agnostisch).
    bool hasMin = !std::isnan(lim.minDeg);
    bool hasMax = !std::isnan(lim.maxDeg);
    float lo, hi;
    if (hasMin && hasMax) { lo = std::fmin(lim.minDeg, lim.maxDeg); hi = std::fmax(lim.minDeg, lim.maxDeg); }
    else if (hasMin)      { lo = lim.minDeg; hi = lim.minDeg; }   // single-side dummy
    else if (hasMax)      { lo = lim.maxDeg; hi = lim.maxDeg; }
    else                  { return r; }

    if (hasMax && currentDeg >= hi) { r.refDeg = currentDeg; r.hitMax = true; }
    if (hasMin && currentDeg <= lo) { r.refDeg = currentDeg; r.hitMin = true; }
    return r;
}

} // namespace soft_endstops
