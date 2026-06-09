#include "encoders.h"
#include <Wire.h>

namespace encoders {

static Channel motorCh_[cfg::NUM_JOINTS];
static Channel armCh_  [cfg::NUM_JOINTS];

static bool tcaSelect(uint8_t channel) {
    Wire.beginTransmission(cfg::TCA_ADDR);
    Wire.write(1 << channel);
    return Wire.endTransmission() == 0;
}

static void tcaClose() {
    Wire.beginTransmission(cfg::TCA_ADDR);
    Wire.write(0);
    Wire.endTransmission();
}

static bool i2cReadAngle12(uint16_t& angle12) {
    constexpr uint8_t AS5600_ADDR    = 0x36;
    constexpr uint8_t AS5600_ANGLE_H = 0x0E;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return false;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    angle12 = ((uint16_t)hi << 8) | lo;
    angle12 &= 0x0FFF;
    return true;
}

void init() {
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        motorCh_[i].muxChannel = cfg::TCA_MOTOR_CHANNELS[i];
        motorCh_[i].dir        = cfg::ENC_DIR;
        armCh_[i].muxChannel   = cfg::TCA_ARM_CHANNELS[i];
        armCh_[i].dir          = cfg::ARM_ENC_DIR;
    }
    Wire.begin();
    tcaClose();
}

bool readRaw(uint8_t muxChannel, uint16_t& out) {
    if (!tcaSelect(muxChannel)) { tcaClose(); return false; }
    bool ok = i2cReadAngle12(out);
    tcaClose();
    return ok;
}

float updateUnwrap(Channel& ch, float newDegRaw) {
    if (!ch.initialised) {
        ch.lastDeg      = newDegRaw;
        ch.unwrappedDeg = 0.0f;
        ch.initialised  = true;
        return ch.unwrappedDeg;
    }
    float d = newDegRaw - ch.lastDeg;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    ch.unwrappedDeg += d;
    ch.lastDeg       = newDegRaw;
    return ch.unwrappedDeg;
}

static bool readAxisImpl(Channel& ch, float& out) {
    uint16_t raw = 0;
    if (!readRaw(ch.muxChannel, raw)) return false;
    out = updateUnwrap(ch, rawToDeg(raw));
    return true;
}

bool readMotorAxis(int jointIdx, float& unwrappedDegOut) {
    if (jointIdx < 0 || jointIdx >= cfg::NUM_JOINTS) return false;
    return readAxisImpl(motorCh_[jointIdx], unwrappedDegOut);
}

bool readArmAxis(int jointIdx, float& unwrappedDegOut) {
    if (jointIdx < 0 || jointIdx >= cfg::NUM_JOINTS) return false;
    return readAxisImpl(armCh_[jointIdx], unwrappedDegOut);
}

Channel& motorChannel(int jointIdx) { return motorCh_[jointIdx]; }
Channel& armChannel  (int jointIdx) { return armCh_  [jointIdx]; }

void zeroAxis(Channel& ch) {
    ch.unwrappedDeg = 0.0f;
    ch.initialised  = false;  // forceer re-init bij volgende read
}

} // namespace encoders
