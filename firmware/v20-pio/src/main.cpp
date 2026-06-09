#include <Arduino.h>
#include "config.h"
#include "led.h"
#include "encoders.h"
#include "tmc_drivers.h"
#include "motion_file.h"
#include "nvs_storage.h"
#include "soft_endstops.h"
#include "pid.h"
#include "commands.h"
#include "recording.h"
#include "playback.h"
#include "homing.h"

// ─── Globale state ─────────────────────────────────────────
enum class AppMode { IDLE, RECORDING, HOMING, PLAYBACK };
static AppMode mode_ = AppMode::IDLE;

static pid::Gains pidGains_[cfg::NUM_JOINTS] = {
    {6.5f, 0.0f, 3.0f},
    {1.4f, 0.05f, 2.5f},   // M2 post-TCA-fix arm-side feedback
    {6.8f, 0.0f, 3.5f},
};
static pid::Config pidCfg_[cfg::NUM_JOINTS] = {};
static soft_endstops::Limits limits_[cfg::NUM_JOINTS] = {};

static int currentPct_[cfg::NUM_JOINTS] = {
    cfg::TMC_DEFAULT_CURRENT_PCT, 40, cfg::TMC_DEFAULT_CURRENT_PCT
};

// ─── Helpers ───────────────────────────────────────────────
static void tmcEdgeReporter(const tmc::EdgeEvent& e) {
    Serial.printf("!! M%d TMC %s t=%lu cs=%u\n", e.joint, e.label, e.millis, e.currentScaling);
}

static void printStatus() {
    const char* modeStr = "?";
    switch (mode_) {
        case AppMode::IDLE:      modeStr = "IDLE"; break;
        case AppMode::RECORDING: modeStr = "RECORDING"; break;
        case AppMode::HOMING:    modeStr = "HOMING"; break;
        case AppMode::PLAYBACK:  modeStr = "PLAYBACK"; break;
    }
    Serial.printf("<OK status mode=%s home=%d motion=%d kp=[%.2f,%.2f,%.2f] ki=[%.3f,%.3f,%.3f] kd=[%.2f,%.2f,%.2f]\n",
        modeStr, 1, motion_file::exists() ? 1 : 0,
        pidGains_[0].kp, pidGains_[1].kp, pidGains_[2].kp,
        pidGains_[0].ki, pidGains_[1].ki, pidGains_[2].ki,
        pidGains_[0].kd, pidGains_[1].kd, pidGains_[2].kd);
}

static void printHelp() {
    Serial.println("<OK help");
    Serial.println("  >HELP                  dit overzicht");
    Serial.println("  >STATUS                mode/PID/gains");
    Serial.println("  >MODE IDLE|RECORDING|HOMING|PLAYBACK");
    Serial.println("  >TUNE KP|KI|KD <i> <v>");
    Serial.println("  >CURRENT <i> <pct>");
    Serial.println("  >HOME");
    Serial.println("  >DEL");
    Serial.println("  >LIMITSET <i> MIN|MAX");
    Serial.println("  >LIMITS                toon endstops");
    Serial.println("  >LIMITSAVE             persist endstops naar NVS");
    Serial.println("  >LIMITCLR <i>          wis endstops voor joint i");
    Serial.println("  >ENCRAW                raw AS5600 hoeken (motor+arm channels)");
    Serial.println("  >TMCSTATUS             TMC2209 DRV_STATUS per motor");
}

static void printLimits() {
    Serial.println("<OK limits");
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        float deg = 0.0f;
        encoders::readArmAxis(i, deg);
        char minBuf[24], maxBuf[24];
        if (std::isnan(limits_[i].minDeg)) snprintf(minBuf, sizeof(minBuf), "(unset)");
        else                                snprintf(minBuf, sizeof(minBuf), "%.2f°", limits_[i].minDeg);
        if (std::isnan(limits_[i].maxDeg)) snprintf(maxBuf, sizeof(maxBuf), "(unset)");
        else                                snprintf(maxBuf, sizeof(maxBuf), "%.2f°", limits_[i].maxDeg);
        Serial.printf("  M%d cur=%.2f°   min=%s   max=%s\n", i+1, deg, minBuf, maxBuf);
    }
}

static void printEncRaw() {
    Serial.println("<OK encraw");
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        uint16_t mRaw = 0, aRaw = 0;
        bool okM = encoders::readRaw(cfg::TCA_MOTOR_CHANNELS[i], mRaw);
        bool okA = encoders::readRaw(cfg::TCA_ARM_CHANNELS  [i], aRaw);
        if (okM && okA) {
            Serial.printf("  M%d motor_ch%u raw=%4u (%.2f°)   arm_ch%u raw=%4u (%.2f°)\n",
                i+1,
                (unsigned)cfg::TCA_MOTOR_CHANNELS[i], (unsigned)mRaw, encoders::rawToDeg(mRaw),
                (unsigned)cfg::TCA_ARM_CHANNELS[i],   (unsigned)aRaw, encoders::rawToDeg(aRaw));
        } else {
            Serial.printf("  M%d read FAIL\n", i+1);
        }
    }
}

static void printTmcStatus() {
    Serial.println("<OK tmcstatus");
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        auto s = tmc::snapshot(i);
        Serial.printf("  M%d ot=%u otpw=%u t120=%u t143=%u t150=%u t157=%u  cs=%u stst=%u stealth=%u  "
                      "ol=%u/%u s2g=%u/%u s2vs=%u/%u  drv_err=%u uv=%u\n",
                      i+1, s.ot, s.otpw, s.t120, s.t143, s.t150, s.t157,
                      s.current_scaling, s.standstill, s.stealth_chop_mode,
                      s.open_load_a, s.open_load_b,
                      s.short_to_ground_a, s.short_to_ground_b,
                      s.low_side_short_a, s.low_side_short_b,
                      s.drv_err, s.uv_cp);
    }
}

static void enterMode(AppMode m) {
    if (mode_ == AppMode::RECORDING) recording::stop();
    if (mode_ == AppMode::PLAYBACK)  playback::stop();
    mode_ = m;
    if (m == AppMode::RECORDING) {
        if (!recording::start()) { Serial.println("<ERR startRecording failed"); mode_ = AppMode::IDLE; return; }
        led::set(led::BLINK_1HZ, CRGB::Red);
    } else if (m == AppMode::PLAYBACK) {
        playback::Config c{};
        for (int i = 0; i < cfg::NUM_JOINTS; i++) {
            c.gains[i]  = pidGains_[i];
            c.pidCfg[i] = pidCfg_[i];
            c.limits[i] = limits_[i];
        }
        homing::initFromVernier();
        if (!playback::start(c)) { Serial.println("<ERR startPlayback failed"); mode_ = AppMode::IDLE; return; }
        led::set(led::BLINK_1HZ, CRGB::Green);
    } else if (m == AppMode::IDLE) {
        tmc::disableAll();
        led::set(led::SOLID, CRGB::Blue);
    } else if (m == AppMode::HOMING) {
        homing::initFromVernier();
        led::set(led::BLINK_5HZ, CRGB::Yellow);
    }
    Serial.printf("<OK mode=%s\n",
        m == AppMode::IDLE ? "IDLE" :
        m == AppMode::RECORDING ? "RECORDING" :
        m == AppMode::HOMING ? "HOMING" : "PLAYBACK");
}

static void handleCommand(const commands::ParsedCommand& pc) {
    using V = commands::Verb;
    if (!pc.valid) { Serial.printf("<ERR %s\n", pc.errorMsg ? pc.errorMsg : "bad command"); return; }
    switch (pc.verb) {
        case V::HELP:      printHelp(); break;
        case V::STATUS:    printStatus(); break;
        case V::MODE: {
            using M = commands::Mode;
            switch (pc.mode) {
                case M::IDLE:      enterMode(AppMode::IDLE); break;
                case M::RECORDING: enterMode(AppMode::RECORDING); break;
                case M::HOMING:    enterMode(AppMode::HOMING); break;
                case M::PLAYBACK:  enterMode(AppMode::PLAYBACK); break;
            }
            break;
        }
        case V::TUNE: {
            using G = commands::GainKey;
            float* dst = nullptr;
            switch (pc.gainKey) {
                case G::KP: dst = &pidGains_[pc.jointIdx].kp; break;
                case G::KI: dst = &pidGains_[pc.jointIdx].ki; break;
                case G::KD: dst = &pidGains_[pc.jointIdx].kd; break;
            }
            *dst = pc.value;
            Serial.printf("<OK tune %c%c[%d]=%.4f\n",
                pc.gainKey == G::KP ? 'K' : pc.gainKey == G::KI ? 'K' : 'K',
                pc.gainKey == G::KP ? 'P' : pc.gainKey == G::KI ? 'I' : 'D',
                pc.jointIdx, pc.value);
            break;
        }
        case V::CURRENT:
            currentPct_[pc.jointIdx] = pc.percent;
            tmc::setCurrentPct(pc.jointIdx, pc.percent);
            Serial.printf("<OK current M%d run=%d%%\n", pc.jointIdx+1, pc.percent);
            break;
        case V::HOME:
            if (homing::snapshotCurrentAsHome()) Serial.println("<OK home saved");
            else                                  Serial.println("<ERR home failed");
            break;
        case V::DEL:
            if (motion_file::remove()) Serial.println("<OK motion file deleted");
            else                       Serial.println("<ERR no motion file");
            break;
        case V::LIMITSET: {
            float deg = 0.0f;
            if (!encoders::readArmAxis(pc.jointIdx, deg)) { Serial.println("<ERR encoder read"); break; }
            if (pc.limitSide == commands::LimitSide::MIN) limits_[pc.jointIdx].minDeg = deg;
            else                                          limits_[pc.jointIdx].maxDeg = deg;
            Serial.printf("<OK lim M%d %s = %.2f° (RAM only — gebruik LIMITSAVE)\n",
                pc.jointIdx+1, pc.limitSide == commands::LimitSide::MIN ? "MIN" : "MAX", deg);
            break;
        }
        case V::LIMITS:    printLimits(); break;
        case V::LIMITSAVE: nvs_storage::saveLimits(limits_); Serial.println("<OK limits saved to NVS"); break;
        case V::LIMITCLR:
            limits_[pc.jointIdx].minDeg = NAN;
            limits_[pc.jointIdx].maxDeg = NAN;
            nvs_storage::clearLimits(pc.jointIdx);
            Serial.printf("<OK limits M%d cleared\n", pc.jointIdx+1);
            break;
        case V::ENCRAW:    printEncRaw(); break;
        case V::TMCSTATUS: printTmcStatus(); break;
        case V::UNKNOWN:   Serial.println("<ERR unknown"); break;
    }
}

static String cmdBuf_;

static void pollSerialCommands() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            String line = cmdBuf_; cmdBuf_ = ""; line.trim();
            if (line.length() > 0) handleCommand(commands::parse(line.c_str()));
        } else if (cmdBuf_.length() < 200) {
            cmdBuf_ += c;
        }
    }
}

// ─── Arduino entry points ──────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("v20-pio — modulair (zie firmware/v20-pio/docs/architecture.md)");
    Serial.println("Type >HELP voor commando-overzicht");

    led::init();
    encoders::init();
    tmc::init();
    for (int i = 0; i < cfg::NUM_JOINTS; i++) tmc::setCurrentPct(i, currentPct_[i]);
    motion_file::init();

    // Herstel home + endstops uit NVS
    nvs_storage::HomePose hp;
    nvs_storage::loadHome(hp);
    nvs_storage::loadLimits(limits_);

    // Set PID config defaults (output clamps op TMC VACTUAL bereik)
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        pidCfg_[i].outputMin = -200000.0f;
        pidCfg_[i].outputMax =  200000.0f;
        pidCfg_[i].deadbandErr = 1.0f;
        pidCfg_[i].iClampAbs = 50000.0f;
    }

    enterMode(AppMode::IDLE);
}

void loop() {
    pollSerialCommands();
    tmc::pollEdgeDetect(tmcEdgeReporter);

    switch (mode_) {
        case AppMode::RECORDING: recording::update(); break;
        case AppMode::PLAYBACK:  playback::update();  break;
        case AppMode::HOMING:    /* nog: niet-blokkerende homing-stap */ break;
        case AppMode::IDLE:      break;
    }

    led::update();
}
