#include "soft_endstops.h"

namespace soft_endstops {

ClampResult clamp(float refDeg, float currentDeg, const Limits& lim) {
    ClampResult r{refDeg, false, false};
    if (!std::isnan(lim.maxDeg) && currentDeg >= lim.maxDeg) {
        r.refDeg = currentDeg;
        r.hitMax = true;
    }
    if (!std::isnan(lim.minDeg) && currentDeg <= lim.minDeg) {
        r.refDeg = currentDeg;
        r.hitMin = true;
    }
    return r;
}

} // namespace soft_endstops
