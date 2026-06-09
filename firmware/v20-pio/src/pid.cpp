#include "pid.h"

namespace pid {

float compute(float error, float dtSec, const Gains& g, const Config& c, State& s) {
    if (dtSec <= 0.0f) return 0.0f;

    if (std::fabs(error) <= c.deadbandErr) {
        // Binnen deadband: geen output, integraal niet laten groeien.
        s.lastError   = error;
        s.initialised = true;
        return 0.0f;
    }

    // Anti-windup: alleen integreren als output niet al gesatureerd is in dezelfde richting.
    float candidateI = s.iAccum + error * dtSec;
    if (candidateI >  c.iClampAbs) candidateI =  c.iClampAbs;
    if (candidateI < -c.iClampAbs) candidateI = -c.iClampAbs;

    float dTerm = 0.0f;
    if (s.initialised) {
        dTerm = (error - s.lastError) / dtSec;
    }

    float out = g.kp * error + g.ki * candidateI + g.kd * dTerm;

    // Output saturation; bij saturatie geen verdere integraal-toename (back-calc light).
    if (out >= c.outputMax) {
        out = c.outputMax;
        if (error * candidateI > 0.0f) candidateI = s.iAccum;  // freeze integrator
    } else if (out <= c.outputMin) {
        out = c.outputMin;
        if (error * candidateI > 0.0f) candidateI = s.iAccum;
    }

    s.iAccum      = candidateI;
    s.lastError   = error;
    s.initialised = true;
    return out;
}

} // namespace pid
