#pragma once
#include <string>

namespace commands {

enum class Verb {
    UNKNOWN,
    HELP,
    STATUS,
    MODE,
    TUNE,
    CURRENT,
    HOME,
    DEL,
    LIMITSET,
    LIMITS,
    LIMITSAVE,
    LIMITCLR,
    ENCRAW,
    TMCSTATUS,
};

enum class Mode      { IDLE, RECORDING, HOMING, PLAYBACK };
enum class GainKey   { KP, KI, KD };
enum class LimitSide { MIN, MAX };

struct ParsedCommand {
    Verb verb = Verb::UNKNOWN;

    // Argumenten — alleen relevant voor de bijbehorende verb
    Mode      mode      = Mode::IDLE;
    GainKey   gainKey   = GainKey::KP;
    LimitSide limitSide = LimitSide::MIN;
    int       jointIdx  = -1;       // 0..2 of -1
    int       percent   = -1;       // CURRENT
    float     value     = 0.0f;     // TUNE

    bool      valid     = false;    // false = niet correct geparset; verb=UNKNOWN dan
    const char* errorMsg = nullptr; // optionele uitleg bij !valid
};

// Pure: parse één lijn (zonder leading '>'). Whitespace ignored aan begin/eind.
ParsedCommand parse(const char* line);

// Convenience: parse vanuit std::string
inline ParsedCommand parse(const std::string& s) { return parse(s.c_str()); }

} // namespace commands
