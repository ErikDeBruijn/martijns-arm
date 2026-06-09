#pragma once
#include <cmath>

namespace pid {

struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct State {
    float iAccum    = 0.0f;
    float lastError = 0.0f;
    bool  initialised = false;
};

struct Config {
    float outputMin   = -1e9f;
    float outputMax   =  1e9f;
    float deadbandErr =  0.0f;   // |err| <= deadbandErr → 0 output, geen accumulatie
    float iClampAbs   =  1e9f;   // anti-windup cap op |iAccum|
};

// Pure step: compute returns nieuwe output, muteert State.
// dtSec moet > 0 zijn; bij dtSec <= 0 wordt 0 teruggegeven en state ongewijzigd.
float compute(float error, float dtSec, const Gains& g, const Config& c, State& s);

inline void reset(State& s) { s = State{}; }

} // namespace pid
