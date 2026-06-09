#pragma once
#include <Arduino.h>
#include "config.h"
#include "nvs_storage.h"

namespace homing {

// Sla huidige raw motor+arm sensor-waardes op als nieuwe home (zoals v19 `>HOME`).
// Vereist successful encoder reads.
bool snapshotCurrentAsHome();

// Vernier: stel armUnwrappedDeg in op de fysieke offset t.o.v. opgeslagen home.
// Gebruikt motor- én arm-encoder samples om correct te disambigueren tussen
// multi-rotatie configs (3.5:1 belt). Wordt aan start van playback aangeroepen.
bool initFromVernier();

// Forceer arm naar home-positie via PID (closed loop op arm-encoder).
// Returnt true zodra alle 3 joints binnen tolerantie zitten.
bool runUntilStable(uint32_t timeoutMs);

} // namespace homing
