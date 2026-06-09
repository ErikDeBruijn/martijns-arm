#include "playback.h"
#include "motion_file.h"
#include "encoders.h"
#include "tmc_drivers.h"

namespace playback {

float catmullRom(float p0, float p1, float p2, float p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    // Catmull-Rom basis matrix (tension = 0.5)
    return 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f*p0 - 5.0f*p1 + 4.0f*p2 - p3) * t2 +
        (-p0 + 3.0f*p1 - 3.0f*p2 + p3) * t3
    );
}

float linearInterp(long aSteps, uint32_t aTimeMs,
                   long bSteps, uint32_t bTimeMs,
                   uint32_t nowMs) {
    if (bTimeMs == aTimeMs) return (float)aSteps;
    // Cast naar int64_t om unsigned-overflow te vermijden als nowMs < aTimeMs.
    float t = (float)((int64_t)nowMs   - (int64_t)aTimeMs) /
              (float)((int64_t)bTimeMs - (int64_t)aTimeMs);
    if (t < 0.0f) t = 0.0f; if (t > 1.0f) t = 1.0f;
    return (float)aSteps + ((float)bSteps - (float)aSteps) * t;
}

namespace {
    struct RingBuf {
        motion_file::Sample s[4];
        int                  fill = 0;   // 0..4
    } samples_;

    Config            cfg_;
    pid::State        pidState_[cfg::NUM_JOINTS];
    long              lastRef_[cfg::NUM_JOINTS]   = {0};
    long              lastEnc_[cfg::NUM_JOINTS]   = {0};
    float             lastErrDeg_[cfg::NUM_JOINTS]= {0};
    uint32_t          startMs_     = 0;
    uint32_t          lastTickMs_  = 0;
    bool              active_      = false;
    bool              eof_         = false;

    bool fetchNext(motion_file::Sample& dst) {
        return motion_file::readNextSample(dst);
    }

    void slide() {
        // shift left: s[0] = s[1], s[1]=s[2], s[2]=s[3]
        samples_.s[0] = samples_.s[1];
        samples_.s[1] = samples_.s[2];
        samples_.s[2] = samples_.s[3];
        samples_.fill = 3;
    }

    long armDegToSteps(float deg) {
        return (long)((deg / 360.0f) * (float)cfg::STEPS_PER_ARM_REV);
    }
    float armStepsToDeg(long steps) {
        return ((float)steps / (float)cfg::STEPS_PER_ARM_REV) * 360.0f;
    }
}

bool start(const Config& c) {
    if (active_) return true;
    if (!motion_file::openRead()) return false;
    cfg_ = c;
    samples_.fill = 0;
    // Prefetch tot 4 samples
    while (samples_.fill < 4) {
        if (!fetchNext(samples_.s[samples_.fill])) { eof_ = true; break; }
        samples_.fill++;
    }
    if (samples_.fill < 2) { motion_file::close(); return false; }
    for (int i = 0; i < cfg::NUM_JOINTS; i++) pid::reset(pidState_[i]);
    startMs_    = millis();
    lastTickMs_ = 0;
    eof_        = false;
    active_     = true;
    return true;
}

void stop() {
    if (!active_) return;
    motion_file::close();
    tmc::disableAll();
    active_ = false;
}

void update() {
    if (!active_) return;
    uint32_t now = millis();
    if (now - lastTickMs_ < cfg::SAMPLE_MS) return;
    uint32_t dtMs = (lastTickMs_ == 0) ? cfg::SAMPLE_MS : (now - lastTickMs_);
    lastTickMs_ = now;

    uint32_t playTime = now - startMs_;

    // Slide window vooruit tot middle sample (s[1]) overeenkomt met of net voor playTime
    while (samples_.fill >= 4 && samples_.s[2].tMs < playTime) {
        slide();
        motion_file::Sample n{};
        if (fetchNext(n)) samples_.s[3] = n; else eof_ = true;
    }

    // Klaar?
    if (samples_.fill < 2 || (eof_ && playTime > samples_.s[samples_.fill - 1].tMs)) {
        stop();
        return;
    }

    // Interpolation between s[1] en s[2]; gebruik s[0] en s[3] als tangenten.
    float t = 0.0f;
    if (samples_.s[2].tMs > samples_.s[1].tMs) {
        t = (float)(playTime - samples_.s[1].tMs) /
            (float)(samples_.s[2].tMs - samples_.s[1].tMs);
        if (t < 0.0f) t = 0.0f; if (t > 1.0f) t = 1.0f;
    }

    float dtSec = dtMs / 1000.0f;
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        float ref;
        if (samples_.fill >= 4) {
            ref = catmullRom((float)samples_.s[0].steps[i],
                             (float)samples_.s[1].steps[i],
                             (float)samples_.s[2].steps[i],
                             (float)samples_.s[3].steps[i], t);
        } else {
            ref = linearInterp(samples_.s[1].steps[i], samples_.s[1].tMs,
                               samples_.s[2].steps[i], samples_.s[2].tMs, playTime);
        }
        long refSteps = (long)ref;

        // Encoder
        float encDeg = 0.0f;
        if (!encoders::readArmAxis(i, encDeg)) continue;
        long  encSteps = armDegToSteps(encDeg);

        // Soft endstop clamp op huidige fysieke positie (arm-graden)
        float refDeg = armStepsToDeg(refSteps);
        auto clamp = soft_endstops::clamp(refDeg, encDeg, cfg_.limits[i]);
        if (clamp.hitMin || clamp.hitMax) {
            refSteps = encSteps;
        }

        // PID op steps-error → motor command
        float errSteps = (float)(refSteps - encSteps);
        float u = pid::compute(errSteps, dtSec, cfg_.gains[i], cfg_.pidCfg[i], pidState_[i]);

        tmc::setVelocity(i, (int32_t)u);

        lastRef_[i]    = refSteps;
        lastEnc_[i]    = encSteps;
        lastErrDeg_[i] = armStepsToDeg((long)errSteps);
    }
}

bool active()                  { return active_; }
long lastRefSteps (int i)      { return (i >= 0 && i < cfg::NUM_JOINTS) ? lastRef_[i] : 0; }
long lastEncSteps (int i)      { return (i >= 0 && i < cfg::NUM_JOINTS) ? lastEnc_[i] : 0; }
float lastErrorDeg(int i)      { return (i >= 0 && i < cfg::NUM_JOINTS) ? lastErrDeg_[i] : 0.0f; }

} // namespace playback
