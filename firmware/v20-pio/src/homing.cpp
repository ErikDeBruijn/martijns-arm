#include "homing.h"
#include "encoders.h"
#include "tmc_drivers.h"
#include <cmath>

namespace homing {

bool snapshotCurrentAsHome() {
    nvs_storage::HomePose hp;
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        if (!encoders::readRaw(cfg::TCA_MOTOR_CHANNELS[i], hp.motorRaw[i])) return false;
        if (!encoders::readRaw(cfg::TCA_ARM_CHANNELS  [i], hp.armRaw  [i])) return false;
    }
    hp.present = true;
    nvs_storage::saveHome(hp);
    // Zero current encoder state to align unwrap with home reference
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        encoders::zeroAxis(encoders::motorChannel(i));
        encoders::zeroAxis(encoders::armChannel  (i));
    }
    return true;
}

bool initFromVernier() {
    nvs_storage::HomePose hp;
    if (!nvs_storage::loadHome(hp) || !hp.present) return false;
    // Vereenvoudigde Vernier: pak arm-raw delta t.o.v. home en zet als unwrappedDeg.
    // Voor M1/M3 met klein werkbereik (< 1 rotatie) is dit voldoende.
    // Voor M2 base yaw (kabel-gelimiteerd) zit het werkbereik binnen 1 arm-rotatie.
    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        uint16_t armRaw = 0;
        if (!encoders::readRaw(cfg::TCA_ARM_CHANNELS[i], armRaw)) return false;
        float curDeg  = encoders::rawToDeg(armRaw);
        float homeDeg = encoders::rawToDeg(hp.armRaw[i]);
        float diff = curDeg - homeDeg;
        if (diff >  180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        auto& ch = encoders::armChannel(i);
        ch.unwrappedDeg = (float)cfg::ARM_ENC_DIR * diff;
        ch.lastDeg      = curDeg;
        ch.initialised  = true;
    }
    return true;
}

bool runUntilStable(uint32_t timeoutMs) {
    // Vereenvoudigde versie: gewoon Vernier-init doen, daarna kleine PID-correctie naar 0°.
    // Voor v20-pio MVP: niet-blokkerend, integratie met PID-loop via playback module gebeurt later.
    // Hier alleen state initialiseren.
    if (!initFromVernier()) return false;

    uint32_t start = millis();
    bool allStable = false;
    while (millis() - start < timeoutMs && !allStable) {
        allStable = true;
        for (int i = 0; i < cfg::NUM_JOINTS; i++) {
            float deg = 0.0f;
            if (!encoders::readArmAxis(i, deg)) { allStable = false; continue; }
            if (std::fabs(deg) > 0.5f) {
                allStable = false;
                // Eenvoudige open-loop nudge richting 0
                int32_t v = (deg > 0) ? -5000 : 5000;
                tmc::setVelocity(i, v);
            } else {
                tmc::setVelocity(i, 0);
            }
        }
        delay(10);
    }
    for (int i = 0; i < cfg::NUM_JOINTS; i++) tmc::setVelocity(i, 0);
    return allStable;
}

} // namespace homing
