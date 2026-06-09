#pragma once
#include <Arduino.h>
#include "config.h"

namespace motion_file {

struct Sample {
    uint32_t tMs;
    long     steps[cfg::NUM_JOINTS];
};

// Pure: parse één CSV-regel "t,s1,s2,s3" → Sample. False bij malformed.
bool parseCsvLine(const char* line, Sample& out);

// File API (ESP32 SD_MMC)
bool init();                         // mount SD
bool exists();
bool remove();
bool openWrite();                    // overschrijft bestaand
bool writeHeader();
bool writeSample(const Sample& s);
void flush();
void close();

bool openRead();
bool readNextSample(Sample& out);    // false bij EOF/fout

} // namespace motion_file
