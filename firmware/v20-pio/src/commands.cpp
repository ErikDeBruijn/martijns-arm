#include "commands.h"
#include <cstring>
#include <cstdlib>
#include <cctype>

namespace commands {

namespace {

// Skip leading whitespace + optional '>' prefix
const char* skipPrefix(const char* p) {
    while (*p && std::isspace((unsigned char)*p)) p++;
    if (*p == '>') p++;
    while (*p && std::isspace((unsigned char)*p)) p++;
    return p;
}

// Lees één whitespace-gescheiden token; advance `cursor` voorbij het token.
// Schrijft het token (uppercased) naar `out` (max outLen-1 chars), nul-terminated.
bool nextToken(const char*& cursor, char* out, int outLen) {
    while (*cursor && std::isspace((unsigned char)*cursor)) cursor++;
    if (!*cursor) return false;
    int n = 0;
    while (*cursor && !std::isspace((unsigned char)*cursor) && n < outLen - 1) {
        out[n++] = (char)std::toupper((unsigned char)*cursor++);
    }
    out[n] = '\0';
    return n > 0;
}

bool streq(const char* a, const char* b) { return std::strcmp(a, b) == 0; }

Verb verbFromStr(const char* s) {
    if (streq(s, "HELP"))       return Verb::HELP;
    if (streq(s, "STATUS"))     return Verb::STATUS;
    if (streq(s, "MODE"))       return Verb::MODE;
    if (streq(s, "TUNE"))       return Verb::TUNE;
    if (streq(s, "CURRENT"))    return Verb::CURRENT;
    if (streq(s, "HOME"))       return Verb::HOME;
    if (streq(s, "DEL"))        return Verb::DEL;
    if (streq(s, "LIMITSET"))   return Verb::LIMITSET;
    if (streq(s, "LIMITS"))     return Verb::LIMITS;
    if (streq(s, "LIMITSAVE"))  return Verb::LIMITSAVE;
    if (streq(s, "LIMITCLR"))   return Verb::LIMITCLR;
    if (streq(s, "ENCRAW"))     return Verb::ENCRAW;
    if (streq(s, "TMCSTATUS"))  return Verb::TMCSTATUS;
    return Verb::UNKNOWN;
}

bool parseInt(const char* s, int& out) {
    if (!*s) return false;
    char* end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (end == s) return false;
    out = (int)v;
    return true;
}

bool parseFloat(const char* s, float& out) {
    if (!*s) return false;
    char* end = nullptr;
    float v = std::strtof(s, &end);
    if (end == s) return false;
    out = v;
    return true;
}

bool isValidJointIdx(int i) { return i >= 0 && i <= 2; }

} // namespace

ParsedCommand parse(const char* line) {
    ParsedCommand pc;
    if (!line) { pc.errorMsg = "null input"; return pc; }

    const char* p = skipPrefix(line);
    char tok[32];
    if (!nextToken(p, tok, sizeof(tok))) { pc.errorMsg = "empty"; return pc; }

    pc.verb = verbFromStr(tok);
    if (pc.verb == Verb::UNKNOWN) { pc.errorMsg = "unknown verb"; return pc; }

    switch (pc.verb) {
        case Verb::HELP: case Verb::STATUS: case Verb::HOME: case Verb::DEL:
        case Verb::LIMITS: case Verb::LIMITSAVE: case Verb::ENCRAW: case Verb::TMCSTATUS:
            pc.valid = true;
            return pc;

        case Verb::MODE: {
            if (!nextToken(p, tok, sizeof(tok))) { pc.errorMsg = "MODE wants IDLE|RECORDING|HOMING|PLAYBACK"; return pc; }
            if      (streq(tok, "IDLE"))      pc.mode = Mode::IDLE;
            else if (streq(tok, "RECORDING")) pc.mode = Mode::RECORDING;
            else if (streq(tok, "HOMING"))    pc.mode = Mode::HOMING;
            else if (streq(tok, "PLAYBACK"))  pc.mode = Mode::PLAYBACK;
            else { pc.errorMsg = "unknown mode"; return pc; }
            pc.valid = true;
            return pc;
        }

        case Verb::TUNE: {
            char gk[8], idxTok[8], valTok[16];
            if (!nextToken(p, gk, sizeof(gk)))     { pc.errorMsg = "TUNE KP|KI|KD <i> <v>"; return pc; }
            if      (streq(gk, "KP")) pc.gainKey = GainKey::KP;
            else if (streq(gk, "KI")) pc.gainKey = GainKey::KI;
            else if (streq(gk, "KD")) pc.gainKey = GainKey::KD;
            else { pc.errorMsg = "TUNE: gain key must be KP|KI|KD"; return pc; }
            if (!nextToken(p, idxTok, sizeof(idxTok)) || !parseInt(idxTok, pc.jointIdx) || !isValidJointIdx(pc.jointIdx)) {
                pc.errorMsg = "TUNE: joint idx 0..2"; return pc;
            }
            if (!nextToken(p, valTok, sizeof(valTok)) || !parseFloat(valTok, pc.value)) {
                pc.errorMsg = "TUNE: bad float value"; return pc;
            }
            pc.valid = true;
            return pc;
        }

        case Verb::CURRENT: {
            char idxTok[8], pctTok[8];
            if (!nextToken(p, idxTok, sizeof(idxTok)) || !parseInt(idxTok, pc.jointIdx) || !isValidJointIdx(pc.jointIdx)) {
                pc.errorMsg = "CURRENT: joint idx 0..2"; return pc;
            }
            if (!nextToken(p, pctTok, sizeof(pctTok)) || !parseInt(pctTok, pc.percent) || pc.percent < 0 || pc.percent > 100) {
                pc.errorMsg = "CURRENT: percent 0..100"; return pc;
            }
            pc.valid = true;
            return pc;
        }

        case Verb::LIMITSET: {
            char idxTok[8], sideTok[8];
            if (!nextToken(p, idxTok, sizeof(idxTok)) || !parseInt(idxTok, pc.jointIdx) || !isValidJointIdx(pc.jointIdx)) {
                pc.errorMsg = "LIMITSET: joint idx 0..2"; return pc;
            }
            if (!nextToken(p, sideTok, sizeof(sideTok))) { pc.errorMsg = "LIMITSET <i> MIN|MAX"; return pc; }
            if      (streq(sideTok, "MIN")) pc.limitSide = LimitSide::MIN;
            else if (streq(sideTok, "MAX")) pc.limitSide = LimitSide::MAX;
            else { pc.errorMsg = "LIMITSET side must be MIN|MAX"; return pc; }
            pc.valid = true;
            return pc;
        }

        case Verb::LIMITCLR: {
            char idxTok[8];
            if (!nextToken(p, idxTok, sizeof(idxTok)) || !parseInt(idxTok, pc.jointIdx) || !isValidJointIdx(pc.jointIdx)) {
                pc.errorMsg = "LIMITCLR: joint idx 0..2"; return pc;
            }
            pc.valid = true;
            return pc;
        }

        case Verb::UNKNOWN:
            return pc;
    }
    return pc;
}

} // namespace commands
