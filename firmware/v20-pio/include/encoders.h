#pragma once
#include <Arduino.h>
#include "config.h"

namespace encoders {

struct Channel {
    uint8_t  muxChannel;
    int      dir;        // +1 of -1
    float    unwrappedDeg = 0.0f;
    float    lastDeg      = 0.0f;
    bool     initialised  = false;
};

void init();  // Wire.begin + TCA reset

// AS5600 raw lezen via TCA9548A op gegeven channel.
// Retourneert 0..4095, of false bij I2C-fout.
bool readRaw(uint8_t muxChannel, uint16_t& out);

// Raw 0..4095 → graden 0..360.
constexpr float rawToDeg(uint16_t raw) {
    return ((float)raw / 4096.0f) * 360.0f;
}

// Pure: update unwrap-state met nieuwe raw reading.
// Past wrap-detectie toe (>180° sprong → ±360 correctie).
// Retourneert de nieuwe unwrappedDeg.
float updateUnwrap(Channel& ch, float newDegRaw);

// Lees motor-as encoder voor joint i; update state; return unwrappedDeg.
// false bij I2C-fout.
bool readMotorAxis(int jointIdx, float& unwrappedDegOut);
bool readArmAxis  (int jointIdx, float& unwrappedDegOut);

// Direct toegang tot state (voor homing, vernier)
Channel& motorChannel(int jointIdx);
Channel& armChannel  (int jointIdx);

// Zero current position als referentie
void zeroAxis(Channel& ch);

} // namespace encoders
