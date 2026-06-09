#include "motion_file.h"
#include <FS.h>
#include <SD_MMC.h>
#include <cstdlib>

namespace motion_file {

static File f_;

bool parseCsvLine(const char* line, Sample& out) {
    if (!line || !*line) return false;
    // Skip eventuele header
    if ((line[0] == 't' || line[0] == 'T') && (line[1] == ',' || line[1] == '_')) return false;

    char* p = const_cast<char*>(line);
    char* end;
    long t = std::strtol(p, &end, 10);
    if (end == p || *end != ',') return false;
    out.tMs = (uint32_t)t;
    p = end + 1;

    for (int i = 0; i < cfg::NUM_JOINTS; i++) {
        long v = std::strtol(p, &end, 10);
        if (end == p) return false;
        out.steps[i] = v;
        p = end;
        if (i < cfg::NUM_JOINTS - 1) {
            if (*p != ',') return false;
            p++;
        }
    }
    return true;
}

bool init() {
    return SD_MMC.begin("/sdcard", true);
}

bool exists()  { return SD_MMC.exists(cfg::MOTION_FILE); }
bool remove()  { return SD_MMC.remove(cfg::MOTION_FILE); }

bool openWrite() {
    if (SD_MMC.exists(cfg::MOTION_FILE)) SD_MMC.remove(cfg::MOTION_FILE);
    f_ = SD_MMC.open(cfg::MOTION_FILE, FILE_WRITE);
    return (bool)f_;
}

bool writeHeader() {
    if (!f_) return false;
    f_.println("t_ms,steps_1,steps_2,steps_3");
    return true;
}

bool writeSample(const Sample& s) {
    if (!f_) return false;
    f_.printf("%lu,%ld,%ld,%ld\n",
              (unsigned long)s.tMs, s.steps[0], s.steps[1], s.steps[2]);
    return true;
}

void flush()   { if (f_) f_.flush(); }
void close()   { if (f_) { f_.close(); f_ = File(); } }

bool openRead() {
    f_ = SD_MMC.open(cfg::MOTION_FILE, FILE_READ);
    return (bool)f_;
}

bool readNextSample(Sample& out) {
    if (!f_) return false;
    while (f_.available()) {
        String line = f_.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        if (parseCsvLine(line.c_str(), out)) return true;
        // headerregel: parse faalt, gewoon door naar volgende
    }
    return false;
}

} // namespace motion_file
