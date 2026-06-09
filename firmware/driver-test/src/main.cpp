// Driver test firmware — geïsoleerde TMC2209 spin sequence per driver.
// Doel: bevestigen dat individuele drivers + bedrading werken met een
// losse 200-step stepper (geen encoder). Open-loop, geen PID, geen NVS.
//
// Pin/UART config = hetzelfde als robot arm v19 (drie drivers delen Serial1).
// Sluit een losse stepper aan op de outputs van één driver per keer.
//
// Commando's via serial (115200 8N1):
//   GO <i>         — voer hele test-sequence uit op driver i (0..2)
//   STEP <i> <n>   — draai driver i precies n full-steps (signed; negatief = links)
//   STOP           — stop alle motors
//   ?              — toon help

#include <Arduino.h>
#include <Wire.h>
#include <TMC2209.h>
#include <QuickPID.h>
#include <sTune.h>

// ─── I2C / AS5600 via TCA9548A (arm-as encoders) ────────────
constexpr uint8_t TCA_ADDR = 0x70;
constexpr uint8_t TCA_ARM_CHANNELS[3] = {1, 3, 6};  // post-fix: M1/M2/M3 arm

// ─── Pin config (identiek aan v19 robot arm) ────────────────
HardwareSerial &TMCSerial = Serial1;
constexpr int UART_RX_PIN = 16;
constexpr int UART_TX_PIN = 17;
constexpr int UART_EN_PIN = 15;

// MS1/MS2 → adres. 1-op-1 met v19's robot arm board (jumpering controleerd).
constexpr TMC2209::SerialAddress TMC_ADDR[3] = {
    TMC2209::SERIAL_ADDRESS_0,  // M1: MS1=GND  MS2=GND
    TMC2209::SERIAL_ADDRESS_1,  // M2: MS1=3V3  MS2=GND
    TMC2209::SERIAL_ADDRESS_2   // M3: MS1=GND  MS2=3V3
};

constexpr int      MICROSTEPS    = 256;
constexpr int      FULL_STEPS    = 200;
constexpr long     MICRO_PER_REV = (long)MICROSTEPS * (long)FULL_STEPS;  // 51200

// Default test currents
constexpr int RUN_CURRENT_PCT  = 60;
constexpr int HOLD_CURRENT_PCT = 40;

// ─── PID closed-loop per joint via QuickPID + unwrap ─────────
// QuickPID gebruikt float pointers (input, output, setpoint). Per joint instance.
// AS5600 raw 0..360° wordt unwrapped naar continuous float zodat D-term niet spike
// rond de 360→0° wrap.
static float pidIn[3]  = {0,0,0};   // unwrapped arm degrees (PID input)
static float pidOut[3] = {0,0,0};   // VACTUAL command (float, cast bij output)
static float pidSp[3]  = {0,0,0};   // setpoint degrees

QuickPID joint0(&pidIn[0], &pidOut[0], &pidSp[0]);
QuickPID joint1(&pidIn[1], &pidOut[1], &pidSp[1]);
QuickPID joint2(&pidIn[2], &pidOut[2], &pidSp[2]);
static QuickPID* joints[3] = { &joint0, &joint1, &joint2 };

// Unwrap state per joint
static bool  unwrapInit[3]   = {false, false, false};
static float unwrapLastDeg[3] = {0,0,0};
static float unwrapBase[3]   = {0,0,0};   // accumulated value at unwrapLastDeg

static bool      pidActive[3]      = {false, false, false};
static int32_t   pidLastVactual[3] = {0,0,0};

// QuickPID heeft geen GetTunings; bewaar gains zelf
static float pidKp[3] = {0,0,0};
static float pidKi[3] = {0,0,0};
static float pidKd[3] = {0,0,0};

// Forward declarations
static bool refreshPidInput(int i);

// Step-response logging per joint
static bool     logActive[3]    = {false, false, false};
static uint32_t logStartMs[3]   = {0,0,0};
static uint32_t logEndMs[3]     = {0,0,0};
static uint32_t logLastMs[3]    = {0,0,0};

// sTune voor autotune (één joint tegelijk).
static int      tuneJoint        = -1;     // -1 = niet actief
static float    tuneIn, tuneOut, tuneSetpoint;
static float    tuneInOffset = 0.0f;  // start-positie; tuner ziet relatieve input (0-start)
static sTune    tuner(&tuneIn, &tuneOut, sTune::ZN_PID, sTune::directIP, sTune::printALL);
static constexpr int32_t PID_VACTUAL_MAX = 140000;

// Update unwrap, return unwrapped degrees.
static float unwrapDeg(int i, float rawDeg) {
    if (!unwrapInit[i]) {
        unwrapInit[i]   = true;
        unwrapLastDeg[i] = rawDeg;
        unwrapBase[i]   = rawDeg;
        return rawDeg;
    }
    float delta = rawDeg - unwrapLastDeg[i];
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    unwrapBase[i]   += delta;
    unwrapLastDeg[i] = rawDeg;
    return unwrapBase[i];
}

TMC2209 drivers[3];

// ─── Encoder helpers ───────────────────────────────────────
static bool tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}

static bool readAS5600Deg(int joint, float& degOut) {
    if (joint < 0 || joint > 2) return false;
    if (!tcaSelect(TCA_ARM_CHANNELS[joint])) return false;
    Wire.beginTransmission(0x36);
    Wire.write(0x0E);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(0x36, 2) != 2) return false;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    uint16_t raw = ((uint16_t)hi << 8 | lo) & 0x0FFF;
    degOut = ((float)raw / 4096.0f) * 360.0f;
    return true;
}

// ─── VACTUAL math ──────────────────────────────────────────
// TMC2209 datasheet: speed = VACTUAL × f_clk / 2^24 microsteps/sec
// Bij interne 12 MHz: 1 VACTUAL = 12_000_000 / 16_777_216 ≈ 0.715 µstep/s
// ⇒ microsteps/sec ≈ VACTUAL × 0.715
// Voor 1 rotatie/sec (51200 µstep/s) → VACTUAL ≈ 71_600
//
// Conservatief default: ~0.5 rot/s = 35000 VACTUAL.
static int32_t g_vactual = 35000;
static int g_run_current = 60;

// Stuur driver `i` aan met snelheid `v` (signed), wacht `ms` ms, stop.
// Defensive: re-init driver config voor elke spin omdat tussenliggende UART writes
// (setRunCurrent, etc.) sneaky state-drift veroorzaken (stealth-mode, microsteps).
static void spinFor(int i, int32_t v, uint32_t ms) {
    drivers[i].setMicrostepsPerStep(256);
    drivers[i].disableStealthChop();
    drivers[i].setRunCurrent((uint8_t)g_run_current);
    drivers[i].enable();
    uint32_t t0 = millis();
    drivers[i].moveAtVelocity(v);
    Serial.printf("  → spin M%d v=%ld ms=%lu (cmd-issued)\n", i+1, (long)v, (unsigned long)ms);
    delay(ms);
    drivers[i].moveAtVelocity(0);
    uint32_t actual = millis() - t0;
    Serial.printf("  ← M%d actual_elapsed=%lums (target=%lums)\n", i+1, (unsigned long)actual, (unsigned long)ms);
    delay(100);   // settling
}

// Draai driver i precies `fullSteps` full-steps (signed).
// Gebruikt g_vactual, berekent tijd uit microsteps.
static void stepFor(int i, long fullSteps) {
    long microsteps = labs(fullSteps) * MICROSTEPS;
    // microsteps/sec ≈ VACTUAL × 0.715  →  sec = µstep / (VACTUAL × 0.715)
    uint32_t ms = (uint32_t)((double)microsteps / (g_vactual * 0.715) * 1000.0);
    int32_t v = (fullSteps >= 0) ? g_vactual : -g_vactual;
    spinFor(i, v, ms);
}

// Hele test-sequence per driver.
static void runSequence(int i) {
    Serial.printf("=== M%d sequence start ===\n", i+1);

    // 1 rondje rechtsom
    Serial.println(" [1/4] 1 rondje rechtsom (200 full steps)");
    stepFor(i, +200);
    delay(800);

    // 1 rondje linksom
    Serial.println(" [2/4] 1 rondje linksom (200 full steps)");
    stepFor(i, -200);
    delay(800);

    // 3 rondjes rechtsom
    Serial.println(" [3/4] 3 rondjes rechtsom (600 full steps)");
    stepFor(i, +600);
    delay(800);

    // 3 rondjes linksom
    Serial.println(" [4/4] 3 rondjes linksom (600 full steps)");
    stepFor(i, -600);

    drivers[i].disable();
    Serial.printf("=== M%d sequence done ===\n", i+1);
}

// ─── Serial command handling ───────────────────────────────
static String cmdBuf;

static void printHelp() {
    Serial.println("─────────────────────────────────────────");
    Serial.println("  Driver test firmware");
    Serial.println("  GO <i>          → run sequence on driver i (0..2)");
    Serial.println("  STEP <i> <n>    → spin driver i exactly n full-steps (signed)");
    Serial.println("  POS <i>         → arm-encoder degrees voor joint i");
    Serial.println("  INIT <i>        → re-init driver config (microsteps/stealth/current)");
    Serial.println("  SETMICRO <i> <n>→ setMicrostepsPerStep (writes UART register)");
    Serial.println("  CURRENT <i> <p> → setRunCurrent percent (0..100)");
    Serial.println("  SPEED <v>       → set VACTUAL voor volgende STEP");
    Serial.println("  DISABLE <i>     → driver disable (motor vrij)");
    Serial.println("  DUMP            → toon settings van alle 3 drivers");
    Serial.println("  STOP            → alle motors disable");
    Serial.println("  ?               → deze help");
    Serial.println("─── PID closed-loop (QuickPID + sTune autotune) ──");
    Serial.println("  SETKP <i> <v>   → P-gain voor joint i");
    Serial.println("  SETKI <i> <v>   → I-gain (gevaarlijk zonder goede P+D!)");
    Serial.println("  SETKD <i> <v>   → D-gain");
    Serial.println("  GOTO <i> <deg>  → PID actief, target=deg (unwrapped)");
    Serial.println("  STEPRESP <i> <delta> → step + log 2s @ 200Hz (CSV: LOG,j,t,err,in,sp,out)");
    Serial.println("  AUTOTUNE <i>    → sTune relay-tune (~5s, gevolgd door GOTO)");
    Serial.println("  PIDOFF          → PID stop, motors disable");
    Serial.println("─────────────────────────────────────────");
}

static void handleCmd(String line) {
    line.trim();
    if (line.startsWith(">")) line = line.substring(1);
    line.trim();
    if (line.length() == 0) return;
    line.toUpperCase();
    if (line == "?") { printHelp(); return; }
    if (line.startsWith("STOP")) {
        for (int i = 0; i < 3; i++) { drivers[i].moveAtVelocity(0); drivers[i].disable(); }
        Serial.println("STOP — all motors disabled");
        return;
    }
    // Helper: parse "<verb> <i> <v>" → idx, val. Returnt false bij parse-fout.
    auto parseTwoArgs = [](const String& s, int prefixLen, int& i, float& v) -> bool {
        int s1 = s.indexOf(' ', prefixLen);
        if (s1 < 0) return false;
        i = s.substring(prefixLen, s1).toInt();
        v = s.substring(s1+1).toFloat();
        return i >= 0 && i <= 2;
    };

    if (line.startsWith("SETKP ") || line.startsWith("SETKI ") || line.startsWith("SETKD ")) {
        int i; float v;
        if (!parseTwoArgs(line, 6, i, v)) { Serial.println("ERR: SETKx <i> <v>"); return; }
        if      (line.startsWith("SETKP")) pidKp[i] = v;
        else if (line.startsWith("SETKI")) pidKi[i] = v;
        else                                pidKd[i] = v;
        joints[i]->SetTunings(pidKp[i], pidKi[i], pidKd[i]);
        Serial.printf("M%d gains: KP=%.3f KI=%.3f KD=%.3f\n", i+1, pidKp[i], pidKi[i], pidKd[i]);
        return;
    }
    if (line.startsWith("GOTO ")) {
        int i; float t;
        if (!parseTwoArgs(line, 5, i, t)) { Serial.println("ERR: GOTO <i> <deg>"); return; }
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
        drivers[i].setRunCurrent((uint8_t)g_run_current);
        drivers[i].enable();
        refreshPidInput(i);
        pidSp[i] = t;
        joints[i]->SetMode(QuickPID::Control::automatic);
        pidActive[i] = true;
        float kp = pidKp[i], ki = pidKi[i], kd = pidKd[i];
        Serial.printf("M%d GOTO sp=%.2f° in=%.2f° (KP=%.3f KI=%.3f KD=%.3f)\n",
            i+1, t, pidIn[i], kp, ki, kd);
        return;
    }
    if (line.startsWith("STEPRESP ")) {
        int i; float d;
        if (!parseTwoArgs(line, 9, i, d)) { Serial.println("ERR: STEPRESP <i> <delta>"); return; }
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
        drivers[i].setRunCurrent((uint8_t)g_run_current);
        drivers[i].enable();
        refreshPidInput(i);
        pidSp[i] = pidIn[i] + d;
        joints[i]->SetMode(QuickPID::Control::automatic);
        pidActive[i]   = true;
        logActive[i]   = true;
        logStartMs[i]  = millis();
        logEndMs[i]    = logStartMs[i] + 2000;
        logLastMs[i]   = 0;
        float kp = pidKp[i], ki = pidKi[i], kd = pidKd[i];
        Serial.printf("# STEPRESP M%d  from=%.2f → to=%.2f  (KP=%.3f KI=%.3f KD=%.3f)\n",
            i+1, pidIn[i] - d, pidSp[i], kp, ki, kd);
        Serial.println("# CSV: LOG,joint,t_ms,err,in,sp,out");
        return;
    }
    if (line.startsWith("AUTOTUNE ")) {
        int i = line.substring(9).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR i"); return; }
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
        drivers[i].setRunCurrent((uint8_t)g_run_current);
        drivers[i].enable();
        refreshPidInput(i);
        // sTune Configure: input/output ranges, target span, settle time, samples, sample time.
        // outputStep = magnitude voor step input. testTimeSec = duration of identification.
        tuner.Reset();
        tuner.Configure(/*inputSpan*/ 360.0f, /*outputSpan*/ 200000.0f,
                        /*outputStart*/ 0.0f, /*outputStep*/ 50000.0f,
                        /*testTimeSec*/ 5,    /*settleTimeSec*/ 1,
                        /*samples*/ 250);
        tuner.SetEmergencyStop(45.0f);  // stop bij 45° relatieve deviatie (safety)
        // Tuner ziet relatieve input: start = 0, eStop check werkt dan op deviatie.
        tuneInOffset = pidIn[i];
        tuneIn = 0.0f;
        tuneOut = 0;
        tuneJoint = i;
        Serial.printf("# AUTOTUNE M%d started — keep arm clear, takes ~5s\n", i+1);
        return;
    }
    if (line.startsWith("PIDOFF")) {
        for (int i = 0; i < 3; i++) {
            if (pidActive[i] || tuneJoint == i) {
                drivers[i].moveAtVelocity(0);
                drivers[i].disable();
                joints[i]->SetMode(QuickPID::Control::manual);
                pidActive[i] = false;
            }
        }
        tuneJoint = -1;
        Serial.println("PID off, motors disabled");
        return;
    }
    if (line.startsWith("POS ")) {
        int i = line.substring(4).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR: i 0..2"); return; }
        float deg = 0.0f;
        if (!readAS5600Deg(i, deg)) { Serial.printf("ERR: encoder read fail M%d\n", i+1); return; }
        Serial.printf("POS M%d arm = %.2f°  (ch%d)\n", i+1, deg, (int)TCA_ARM_CHANNELS[i]);
        return;
    }
    if (line.startsWith("DISABLE ")) {
        int i = line.substring(8).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR"); return; }
        drivers[i].disable();
        Serial.printf("M%d disabled\n", i+1);
        return;
    }
    if (line.startsWith("INIT ")) {
        int i = line.substring(5).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR: i must be 0..2"); return; }
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
        drivers[i].setRunCurrent((uint8_t)g_run_current);
        delay(20);
        Serial.printf("M%d INIT done\n", i+1);
        return;
    }
    if (line.startsWith("SETMICRO ")) {
        int s1 = line.indexOf(' ', 9);
        if (s1 < 0) { Serial.println("ERR: SETMICRO <i> <n>"); return; }
        int i = line.substring(9, s1).toInt();
        int n = line.substring(s1 + 1).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR: i must be 0..2"); return; }
        drivers[i].setMicrostepsPerStep(n);
        delay(20);
        uint16_t actual = drivers[i].getMicrostepsPerStep();
        Serial.printf("M%d setMicrosteps(%d) → readback=%u\n", i+1, n, (unsigned)actual);
        return;
    }
    if (line.startsWith("SPEED ")) {
        long v = line.substring(6).toInt();
        g_vactual = (int32_t)v;
        Serial.printf("g_vactual = %ld\n", (long)g_vactual);
        return;
    }
    if (line.startsWith("CURRENT ")) {
        int s1 = line.indexOf(' ', 8);
        if (s1 < 0) { Serial.println("ERR: CURRENT <i> <pct>"); return; }
        int i = line.substring(8, s1).toInt();
        int pct = line.substring(s1 + 1).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR: i must be 0..2"); return; }
        if (pct < 0 || pct > 100) { Serial.println("ERR: pct 0..100"); return; }
        drivers[i].setRunCurrent((uint8_t)pct);
        drivers[i].setHoldCurrent((uint8_t)(pct * 2 / 3));
        Serial.printf("M%d run=%d%% hold=%d%%\n", i+1, pct, pct*2/3);
        return;
    }
    if (line.startsWith("DUMP")) {
        for (int i = 0; i < 3; i++) {
            uint16_t mres = drivers[i].getMicrostepsPerStep();
            auto s = drivers[i].getSettings();
            uint8_t v = drivers[i].getVersion();
            Serial.printf("  M%d addr=%u version=0x%02X microsteps=%u  run=%u%% hold=%u%%  stealth=%u  comm=%u\n",
                i+1, (unsigned)TMC_ADDR[i], (unsigned)v,
                (unsigned)mres,
                (unsigned)s.irun_percent, (unsigned)s.ihold_percent,
                (unsigned)s.stealth_chop_enabled,
                (unsigned)drivers[i].isSetupAndCommunicating());
        }
        return;
    }
    if (line.startsWith("GO ")) {
        int i = line.substring(3).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR: i must be 0..2"); return; }
        runSequence(i);
        return;
    }
    if (line.startsWith("STEP ")) {
        int s1 = line.indexOf(' ', 5);
        if (s1 < 0) { Serial.println("ERR: STEP <i> <n>"); return; }
        int i = line.substring(5, s1).toInt();
        long n = line.substring(s1 + 1).toInt();
        if (i < 0 || i > 2) { Serial.println("ERR: i must be 0..2"); return; }
        Serial.printf("STEP M%d  full_steps=%ld\n", i+1, n);
        stepFor(i, n);
        // Hold-current blijft actief (geen disable) → motor houdt positie vast voor metingen
        return;
    }
    Serial.print("ERR: unknown command: "); Serial.println(line);
}

static void pollSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') { handleCmd(cmdBuf); cmdBuf = ""; }
        else if (cmdBuf.length() < 200) cmdBuf += c;
    }
}

// ─── Arduino entry points ──────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("driver-test firmware — TMC2209 isolated spin");

    Wire.begin();
    pinMode(UART_EN_PIN, OUTPUT);
    digitalWrite(UART_EN_PIN, LOW);  // global enable

    for (int i = 0; i < 3; i++) {
        drivers[i].setup(TMCSerial, 115200, TMC_ADDR[i], UART_RX_PIN, UART_TX_PIN);
        drivers[i].setHardwareEnablePin(UART_EN_PIN);
        drivers[i].setRunCurrent(RUN_CURRENT_PCT);
        drivers[i].setHoldCurrent(HOLD_CURRENT_PCT);
        drivers[i].setMicrostepsPerStep(MICROSTEPS);
        drivers[i].enableAutomaticCurrentScaling();
        drivers[i].enableAutomaticGradientAdaptation();
        drivers[i].disableStealthChop();      // spreadCycle (sterker, hoorbaarder)
        drivers[i].setStandstillMode(TMC2209::FREEWHEELING);
        drivers[i].disable();                  // standaard uit; per test enable
        delay(50);
        if (!drivers[i].isSetupAndCommunicating()) {
            Serial.printf("M%d (addr%d): GEEN COMMUNICATIE!\n", i+1, (int)TMC_ADDR[i]);
        } else {
            // Lees terug wat de chip aan microsteps denkt te hebben
            uint16_t mres = drivers[i].getMicrostepsPerStep();
            auto s  = drivers[i].getSettings();
            Serial.printf("M%d (addr%d): OK  microsteps=%u  run_cur=%u%%  hold_cur=%u%%  stealth=%u  irun=%u\n",
                i+1, (int)TMC_ADDR[i],
                (unsigned)mres,
                (unsigned)s.irun_percent,
                (unsigned)s.ihold_percent,
                (unsigned)s.stealth_chop_enabled,
                (unsigned)s.irun_register_value);
        }
    }

    // QuickPID configuratie per joint
    for (int i = 0; i < 3; i++) {
        joints[i]->SetOutputLimits(-(float)PID_VACTUAL_MAX, (float)PID_VACTUAL_MAX);
        joints[i]->SetSampleTimeUs(2000);  // 500 Hz
        joints[i]->SetTunings(0.0f, 0.0f, 0.0f);  // begin met alles 0; SETKx live
        joints[i]->SetMode(QuickPID::Control::manual);
        joints[i]->SetProportionalMode(QuickPID::pMode::pOnError);
        joints[i]->SetDerivativeMode(QuickPID::dMode::dOnMeas);   // D op meting, niet error
        joints[i]->SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
    }

    printHelp();
}

// Update pidIn[i] uit AS5600 + unwrap. Returnt false bij encoder-fout.
static bool refreshPidInput(int i) {
    float rawDeg = 0.0f;
    if (!readAS5600Deg(i, rawDeg)) return false;
    pidIn[i] = unwrapDeg(i, rawDeg);
    return true;
}

static void pushVactual(int i, float out) {
    int32_t v = (int32_t)out;
    if (v >  PID_VACTUAL_MAX) v =  PID_VACTUAL_MAX;
    if (v < -PID_VACTUAL_MAX) v = -PID_VACTUAL_MAX;
    if (v != pidLastVactual[i]) {
        drivers[i].moveAtVelocity(v);
        pidLastVactual[i] = v;
    }
}

static void pidServiceJoint(int i) {
    if (!pidActive[i] && tuneJoint != i) return;
    if (!refreshPidInput(i)) return;

    if (tuneJoint == i) {
        // Autotune via sTune — relatieve input (0-start) zodat eStop op deviatie werkt
        tuneIn = pidIn[i] - tuneInOffset;
        if (tuner.Run() != tuner.tunings) {
            pushVactual(i, tuneOut);
        } else {
            // Tuning klaar: lees uit en zet op joint
            float kp, ki, kd;
            tuner.GetAutoTunings(&kp, &ki, &kd);
            joints[i]->SetTunings(kp, ki, kd);
            joints[i]->SetMode(QuickPID::Control::automatic);
            Serial.printf("# AUTOTUNE done M%d: KP=%.3f KI=%.3f KD=%.3f\n",
                          i+1, kp, ki, kd);
            tuneJoint   = -1;
            pidActive[i] = true;
        }
    } else if (joints[i]->Compute()) {
        pushVactual(i, pidOut[i]);
    }

    // Step-response logging
    uint32_t now = millis();
    if (logActive[i]) {
        if (now >= logEndMs[i]) {
            logActive[i] = false;
            Serial.println("# step-response logging done");
        } else if (now - logLastMs[i] >= 5) {
            logLastMs[i] = now;
            float err = pidSp[i] - pidIn[i];
            Serial.printf("LOG,M%d,t=%lu,err=%.2f,in=%.2f,sp=%.2f,out=%.0f\n",
                i+1, (unsigned long)(now - logStartMs[i]),
                err, pidIn[i], pidSp[i], pidOut[i]);
        }
    }
}

void loop() {
    pollSerial();
    static uint32_t lastPidMs = 0;
    uint32_t now = millis();
    if (now - lastPidMs >= 2) {   // 500 Hz service
        lastPidMs = now;
        for (int i = 0; i < 3; i++) pidServiceJoint(i);
    }
}
