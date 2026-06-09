// ============================================================
//  Robotarm Teach & Repeat  –  v19  (3 motoren, 3.5:1 tandriem)
//
//  v19: Serial-command parser bovenop v18.1 — voorbereiding ROS2 brug.
//    Functioneel gedrag identiek aan v18.1 (zelfde knoppen, zelfde PID,
//    zelfde streaming PB-output). Toegevoegd: ASCII-commando's via Serial.
//
//    Protocol:
//      Host -> ESP32:  ">VERB [ARGS...]\n"
//      ESP32 -> Host:  "<OK ..." of "<ERR ..."
//      (PB streaming output blijft ongewijzigd)
//
//    Commando's:
//      >HELP                     toon dit overzicht
//      >STATUS                   one-shot status (mode, home, motion file)
//      >MODE IDLE|HOMING|PLAYBACK|RECORDING
//                                wissel mode (zelfde safety-checks als knoppen)
//      >TUNE KP|KI|KD <i> <v>    live PID-tune (i=0..2, v=float). Direct effect.
//      >CURRENT <i> <pct>        live TMC run-current 0..100% (i=0..2). Direct effect.
//      >HOME                     sla huidige positie op als home (= lang BTN_PLAY)
//      >DEL                      verwijder motion file
//      >LIMITSET <i> MIN|MAX     leg huidige arm-positie vast als endstop
//      >LIMITS                   toon alle endstops + huidige posities
//      >LIMITSAVE                schrijf endstops naar NVS (persistent)
//      >LIMITCLR <i>             verwijder endstops voor joint i (= lang BTN_DEL)
//
//    Veiligheid: alle MODE-overgangen lopen via dezelfde guards als de knop-
//    handlers (homeIsSet, motion file bestaat, etc.). Onbekende of mis-
//    gevormde commando's geven "<ERR ..." en hebben geen effect.
//
//    PB-stream blijft het primaire feedback-kanaal. >STATUS is voor
//    interactief debug, niet als datakanaal.
//  ----------------------------------------------------------
//  v16: Aangepast voor nieuw tandriem-hardware ontwerp
//    Gebaseerd op v14.6 (werkende basis, niet v15)
//    Wijzigingen t.o.v. v14.6:
//      • TCA_CHANNELS  gewijzigd: motor-encoders nu mux {0, 3, 7}
//      • TCA_ARM_CHANNELS gewijzigd: arm-encoders nu mux {1, 2, 6}
//      • M1/M2/M3 = ALLEMAAL 200-staps (1.8°/step) — bevestigd via encoder-meting
//        op driver-test firmware 2026-06-09 met arm-AS5600. Vorige comment ("M2/M3 = 400-staps")
//        was FOUT. MOTOR_VACTUAL_CORR aanpassen indien nog op 0.5f voor M2/M3 staat.
//        MOTOR_VACTUAL_CORR[0]=0.5 compenseert verschil in microstaps/omwenteling:
//        v18.0 aanname: M1=51200, M2/M3=102400 µsteps/rev  ← onjuist
//        v18.1: M2+M3 zijn ook Kysan 1124090 (200-step) → MOTOR_VACTUAL_CORR[1/2] = 0.5f
//        Bij gelijke VACTUAL-waarde draait een 200-step motor fysiek 2× zo snel → halveer VACTUAL.
//        Alle PID-rekening (arm encoder steps) is onveranderd — alleen VACTUAL output.
//      • Knoppen ongewijzigd (BTN_REC=4, BTN_PLAY=2, BTN_DEL=1)
//      • Alle andere parameters identiek aan v14.6
//
//  v14.6: GLOBALE PLAYBACK-SNELHEID (trager = betere pad-volging)
//    Gebaseerd op v14.5 diagnostiek:
//      - Cirkel-tracking zelf was al goed (<1° arm error)
//      - Grote errors zaten in snelle stukken waar opname te snel was
//      - User-wens: "langzamer is ok mits precies op de lijn en vloeiend"
//    Oplossing: GLOBAL_PLAYBACK_SPEED factor op virtualMs-update.
//      - Alle snelheden/accelleraties schalen lineair mee
//      - refVel wordt automatisch kleiner (afgeleide van refF t.o.v. echte tijd)
//      - Time-scaling pomp wordt zelden meer getriggerd
//    v14.5 time-scaling parameters (MIN_TIME_SCALE=0.70, PEAK_FOLLOW=2400)
//    BLIJVEN AAN — bij trage playback is pomp al minder waarschijnlijk,
//    maar voor het geval encoder-ruis nog triggert, dempen deze waarden.
//    Diagnostiek (per-as accumulators + eindsamenvatting) blijft AAN.
//  v14.5: TIME-SCALING DEMPING (gerichte fix gebaseerd op v14.4 metingen)
//    Diagnostiek v14.4 toonde aan:
//      - H1 (clipping) verworpen: 0% clipping tijdens cirkel
//      - H3 (M2 slechtste as) omgekeerd: M2 is juist het beste
//      - Time-scaling lus duikte regelmatig naar sc=0.6-0.7 tijdens cirkel
//    Twee parameter-wijzigingen, samen gericht op time-scaling-overgevoeligheid:
//      - PEAK_FOLLOW_ERROR_STEPS: 720 → 2400 (~3× minder gevoelig op trage delen)
//      - MIN_TIME_SCALE: 0.35 → 0.70 (vertraging gecapt op 30% i.p.v. 65%)
//    Alle andere parameters identiek aan v14.3/v14.4.
//    Diagnostiek (per-as accumulators + eindsamenvatting) blijft AAN.
//  v14.4: DIAGNOSTIEK-ONLY BUILD (geen gedragsverandering t.o.v. v14.3)
//    Doel: meten welke van drie hypothesen de pad-volging verklaart.
//      H1: speed-clipping (speedCmd raakt MAX_SPEED)
//      H2: velocity-feedforward mismatch (structurele lag op snelle stukken)
//      H3: as-asynchroniteit (M2 met KD=10 reageert anders dan M1/M3)
//    Toegevoegd:
//      - Per-as accumulators tijdens playback: max|error|, max|cmdPre|, clipTicks, totalTicks
//      - Realtime regel uitgebreid met clip-flag en cmd-fractie per as
//      - Optionele dichte CSV-stream via #define DENSE_CSV_LOG (1=aan)
//      - Eindsamenvatting bij stopPlayback
//    NIETS in het regelgedrag is gewijzigd. Parameters identiek aan v14.3.
//  Gebaseerd op v13.0.
//  v14.0: AccelStepper vervangen door TMC2209 UART/VACTUAL
//    Motoraansturing via driver.moveAtVelocity() i.p.v. STEP/DIR pulsen.
//    Drivers aangestuurd via HardwareSerial(1) — pins aan setup() meegeven.
//    EN via driver.enable()/disable() — geen hardware EN-pin meer nodig.
//    STEPS_PER_REV: 102400 (256 microsteps × 400 fullsteps).
//    PID-gains, toleranties en CSV-eenheid NIET compatibel met v13 opnames.
//  ─────────────────────────────────────────────────────────
//  UART bedrading:
//    GPIO 17  →  1kΩ  →  UART-pin M1/M2/M3 (gedeeld)
//    GPIO 16  →          UART-pin M1/M2/M3 (zelfde node)
//    GPIO 15  →          EN-pin  M1/M2/M3  (gedeeld)
//  Adressering via MS1/MS2 (gelatch bij power-on):
//    M1: MS1=GND MS2=GND  →  adres 0
//    M2: MS1=3V3 MS2=GND  →  adres 1
//    M3: MS1=GND MS2=3V3  →  adres 2
//  ─────────────────────────────────────────────────────────
//  Vervallen t.o.v. v13:
//    - AccelStepper library
//    - M1/M2/M3 STEP, DIR pin definities (GPIO 13/11/12, 18/17/16)
//    - steppers[].setSpeed() / runSpeed()
//    - 3ms stepper-run blok in loop()
//  Nieuw t.o.v. v13:
//    + TMC2209 library (Janelia/Peter Polidoro)
//    + HardwareSerial TMCSerial(1)
//    + TMC2209 drivers[3] met UART adressering
//    + STEPS_PER_REV = 102400
//    + VACTUAL_SCALE: omrekening steps/s → VACTUAL eenheid
//    + setMotorSpeed() vervangt steppers[i].setSpeed()
//  ─────────────────────────────────────────────────────────
//  Benodigde libraries:
//    TMC2209  by Peter Polidoro  (via Library Manager: "TMC2209 Janelia")
//    FastLED  (via Library Manager)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <TMC2209.h>
#include <Preferences.h>
#include <FastLED.h>
#include "FS.h"
#include "SD_MMC.h"

// PIO-only: forward declarations (Arduino IDE genereert ze automatisch)
bool initSD();
bool startHoming();
bool startRecording();
void stopRecording();
bool startPlayback();
void stopPlayback();
void deleteMotionFile();
void handleHoming();
void handleRecording();
void handlePlayback();
bool readArmEncoderSteps(int idx, long &stepsOut);
bool readEncoderSteps(int idx, long &stepsOut);
bool readRawAngle(int idx, uint16_t &raw);
bool readRawArmAngle(int idx, uint16_t &raw);
bool zeroEncoder(int idx);
bool zeroArmEncoder(int idx);
void updateLedForMode();
void setMotorSpeed(int idx, float steps_per_s);
void loadHomeFromNVS();
void saveHomeToNVS(uint16_t rawMotor[3], uint16_t rawArm[3]);
bool initFromVernier();

// ─────────────────────────── UART / TMC2209 ─────────────────
// Alle drie drivers delen één UART-lijn (Serial1)
// Pins worden doorgegeven aan driver.setup() — geen Serial1.begin() nodig
#define UART_TX_PIN   17    // → 1kΩ → UART-pin M1/M2/M3
#define UART_RX_PIN   16    // → UART-pin M1/M2/M3 (zelfde node)
#define UART_EN_PIN   15    // → EN-pin M1/M2/M3

#define MOTOR_ON   LOW      // EN actief laag (hardware EN pin, als fallback)
#define MOTOR_OFF  HIGH

// ─────────────────────────── I2C / TCA9548A ─────────────────
#define SDA_PIN      8
#define SCL_PIN      9
#define TCA_ADDR     0x70
#define AS5600_ADDR  0x36
#define REG_STATUS   0x0B
#define REG_ANGLE_H  0x0E
#define REG_ANGLE_L  0x0F

// Motor-as encoders — v16: aangepast voor nieuw hardware ontwerp
// Bevestigd via mux-scan (motor_resolution_test_v2)
// v19 fix (2026-06-09): M2 motor en arm-kanalen waren in firmware omgewisseld
// t.o.v. fysieke bekabeling. Bevestigd door Erik via TCA-bedrading check:
//   ch0=M1 motor, ch1=M1 arm, ch2=M2 motor, ch3=M2 arm, ch6=M3 arm, ch7=M3 motor
// Verklaart waarom M2's "armUnwrappedDeg" met GEAR_RATIO (3.5×) afwijking gaf.
static const uint8_t TCA_CHANNELS[3]     = {0, 2, 7};  // M1=ch0, M2=ch2, M3=ch7  (motor)

// Arm-as encoders — v16: aangepast voor nieuw hardware ontwerp
static const uint8_t TCA_ARM_CHANNELS[3] = {1, 3, 6};  // M1=ch1, M2=ch3, M3=ch6  (arm)

// ─────────────────────────── KNOPPEN ────────────────────────
#define BTN_REC   4
#define BTN_PLAY  2
#define BTN_DEL   1

// ─────────────────────────── SD-KAART ───────────────────────
static const int SD_CLK = 38;
static const int SD_CMD = 34;
static const int SD_D0  = 39;
static const int SD_D1  = 40;
static const int SD_D2  = 47;
static const int SD_D3  = 33;
static const int SD_DET = 48;

// ─────────────────────────── RGB LED ────────────────────────
#define LED_PIN         46
#define LED_CHIPSET     WS2812
#define LED_COLOR_ORDER GRB
#define LED_BRIGHTNESS  80
#define NUM_LEDS        1

CRGB leds[NUM_LEDS];

enum LedAnim {
    LED_OFF, LED_SOLID, LED_PULSE_SLOW, LED_PULSE_FAST,
    LED_BLINK_1HZ, LED_BLINK_5HZ, LED_PROGRESS, LED_FLASH3,
};

struct LedState {
    LedAnim  anim       = LED_OFF;
    CRGB     color      = CRGB::Black;
    float    progress   = 0.0f;
    uint32_t lastMs     = 0;
    uint8_t  flashCount = 0;
    bool     flashOn    = false;
    bool     flashDone  = false;
} ledState;

void ledSet(LedAnim anim, CRGB color, float progress = 0.0f) {
    ledState.anim       = anim;
    ledState.color      = color;
    ledState.progress   = progress;
    ledState.lastMs     = millis();
    ledState.flashCount = 0;
    ledState.flashOn    = false;
    ledState.flashDone  = false;
}

void ledUpdate() {
    uint32_t dt = millis() - ledState.lastMs;
    uint8_t  br = 0;
    switch (ledState.anim) {
        case LED_OFF:   leds[0] = CRGB::Black; break;
        case LED_SOLID: leds[0] = ledState.color; break;
        case LED_PULSE_SLOW: {
            float s = sinf(((float)(dt % 2000) / 2000.0f) * 2.0f * PI);
            leds[0] = ledState.color; leds[0].nscale8((uint8_t)(s*s*200.0f+10.0f)); break;
        }
        case LED_PULSE_FAST: {
            float s = sinf(((float)(dt % 500) / 500.0f) * 2.0f * PI);
            leds[0] = ledState.color; leds[0].nscale8((uint8_t)(s*s*200.0f+20.0f)); break;
        }
        case LED_BLINK_1HZ:
            leds[0] = ((dt % 1000) < 500) ? ledState.color : CRGB::Black; break;
        case LED_BLINK_5HZ:
            leds[0] = ((dt % 200) < 100)  ? ledState.color : CRGB::Black; break;
        case LED_PROGRESS: {
            br = (uint8_t)(ledState.progress * 220.0f + 10.0f);
            leds[0] = ledState.color; leds[0].nscale8(br); break;
        }
        case LED_FLASH3: {
            if (ledState.flashDone) { leds[0] = ledState.color; break; }
            bool on = ((dt % 240) < 120);
            if (on != ledState.flashOn) {
                ledState.flashOn = on;
                if (!on) ledState.flashCount++;
            }
            if (ledState.flashCount >= 3) { ledState.flashDone = true; leds[0] = ledState.color; }
            else leds[0] = on ? ledState.color : CRGB::Black;
            break;
        }
    }
    FastLED.show();
}

// ─────────────────────────── CONFIG ─────────────────────────
// === DIAGNOSTIEK v14.4 ===
// Zet op 1 voor dichte CSV-stream tijdens playback (1 regel per CTRL_MS = 200Hz).
// Format: PBCSV,t_ms,scale,ref0,enc0,err0,cmdPre0,spd0,ref1,enc1,err1,cmdPre1,spd1,ref2,enc2,err2,cmdPre2,spd2
// Capture met serial logger en plot in spreadsheet/Python.
#define DENSE_CSV_LOG 0

// === v14.6 GLOBALE PLAYBACK-SNELHEID ===
// Factor waarmee virtualMs groeit. 1.0 = originele opname-snelheid.
// Lager = trager afspelen, betere pad-volging maar langere duur.
// Een cirkel van 15s opname wordt bij 0.4 afgespeeld in 37.5s wall-clock.
static const float PLAYBACK_SPEED = 0.15f;

// 256 microsteps × 400 fullsteps = 102400 steps/motoromwenteling
static const long     STEPS_PER_REV          = 102400;
static const float    GEAR_RATIO             = 3.5f;
static const int      ENC_DIR                = +1;
static const int      ARM_ENC_DIR            = -1;
static const float    ARM_ENC_SCALE          = (STEPS_PER_REV * GEAR_RATIO) / 4096.0f;
static const uint32_t SAMPLE_MS              = 10;
static const uint32_t CTRL_MS                = 5;
static const uint32_t PRINT_MS               = 50;

// VACTUAL schaal: VACTUAL eenheid = steps/s × (2^24 / 12_000_000)
// 2^24 / 12e6 ≈ 1.3981  →  afgerond naar 1.398
static const float    VACTUAL_SCALE          = 1.398f;

// TMC2209 stroom (0–100%)
static const uint8_t  TMC_RUN_CURRENT        = 60;   // verlaagd van 80 — aanpassen per motor
static const uint8_t  TMC_HOLD_CURRENT       = 0;    // 0% bij stilstand — FREEWHEELING doet de rest

static const float REC_FILTER_ALPHA          = 0.85f;
static const float PID_FILTER_ALPHA          = 0.25f;   // meer smoothing op arm-encoder → minder schokkerige playback
static const float DERR_FILTER_ALPHA         = 0.18f;   // low-pass op D-term → minder encoder-ruis in speed command
static const float TS_FILTER_ALPHA           = 0.20f;   // smoothing op globale playback timescale
static const float MIN_TIME_SCALE            = 0.70f;   // v14.5: was 0.35 — minder agressieve vertraging
static const long  REC_HOME_TOLERANCE_STEPS  = 100;

// Vernier: maximaal verschil tussen gemeten en verwachte motor-
// sensorwaarde waarbij een hypothese als 'juist' wordt beschouwd.
// De twee hypothesen zijn 180° gescheiden, dus 45° is een veilige
// drempel — ruim genoeg voor sensorruis, ruim onder de 90° grens.
static const float VERNIER_MATCH_THRESHOLD_DEG = 45.0f;

// v18: M2 PID aangepast voor tandriemen (geen speling meer)
// v14.6: KP=2.7 KD=10.0 was getuned voor tandwiel-speling
// Met tandriemen: KD=10 veroorzaakt oscillatie → verlaagd naar 4.0
// KP verhoogd van 2.7 naar 4.0 voor betere responsiviteit zonder speling
static float KP_SPEED[3] = { 6.5f,  3.0f,  6.8f };   // M2 terug naar 3.0
static float KI_SPEED[3] = { 0.0f,  0.05f, 0.0f };   // M2 kleine I
static float KD_SPEED[3] = { 3.0f,  2.5f,  3.5f };   // M2 ook terug
static float KP_HOME  = 4.5f;
static float KD_HOME  = 4.0f;

// Snelheidslimieten in steps/s (bij 51200 steps/motoromw)
static const float    MAX_HOME_SPEED         = 16000.0f;   // was 1000 bij 6400
static const long     HOME_TOLERANCE_STEPS   = 96;        // ~0.006° arm (was 6)
static const uint32_t HOME_STABLE_MS         = 150;
static const long     MAX_FOLLOW_ERROR_STEPS = 36000;
static const long     PEAK_FOLLOW_ERROR_STEPS= 2400;      // v14.5: was 720 — time-scaling minder gevoelig voor kleine fouten
static const float    KP_PEAK_MAX            = 1.3f;
static const long     DB_OFF_STEPS           = 48;        // was 3
static const long     DB_ON_STEPS            = 96;        // was 6
static const float    DB_VEL_THRESHOLD       = 1280.0f;    // was 80
static const float    MIN_SPEED              = 480.0f;
static const float    MAX_SPEED_STEPS_S      = 140000.0f; // extra marge voor de onderkant van de cirkel (M3 liep hier nog achter)
static const float    ACCEL_STEPS_S2         = 650000.0f;
static const float    DECEL_MULTIPLIER       = 1.0f;
static const uint32_t DEL_HOLD_MS            = 800;
static const uint32_t HOME_HOLD_MS           = 800;

static const char* MOTION_FILE   = "/motion.csv";
static const char* NVS_NS        = "robotarm";
static const char* NVS_KEY_SET   = "homeSet";
// Motor-as home: "homeRaw0"  .. "homeRaw2"
// Arm-as home:   "homeArm0"  .. "homeArm2"  (v11 nieuw)

// ─────────────────────────── GLOBALS ────────────────────────
// ─────────────────────────── TMC2209 DRIVERS ────────────────
HardwareSerial TMCSerial(1);   // Serial1 — pins via driver.setup()

TMC2209 drivers[3];
const TMC2209::SerialAddress TMC_ADDR[3] = {
    TMC2209::SERIAL_ADDRESS_0,   // M1: MS1=GND MS2=GND
    TMC2209::SERIAL_ADDRESS_1,   // M2: MS1=3V3 MS2=GND
    TMC2209::SERIAL_ADDRESS_2,   // M3: MS1=GND MS2=3V3
};
float currentSpeed[3] = {0.0f, 0.0f, 0.0f};  // bijgehouden snelheid in steps/s

Preferences prefs;

enum Mode { IDLE, RECORDING, HOMING, PLAYBACK };
Mode mode = IDLE;

// v19: soft endstops in arm-degrees (NaN = niet ingesteld)
static float lim_min_deg[3] = {NAN, NAN, NAN};
static float lim_max_deg[3] = {NAN, NAN, NAN};

uint16_t homeAngleRaw[3] = {0, 0, 0};   // motor-as home (NVS)
uint16_t homeArmRaw[3]   = {0, 0, 0};   // arm-as home   (NVS) — v11 nieuw
bool     homeIsSet        = false;

bool encValidFromHome[3]  = {false, false, false};
bool homingUseMultiTurn   = false;
long recStartSteps[3]     = {0, 0, 0};

struct MotorCtrl {
    float lastDeg            = 0.0f;
    float unwrappedDeg       = 0.0f;
    long  encZeroOffsetSteps = 0;
    long  lastError          = 0;
    float dErrFilt           = 0.0f;
    bool  inDeadBand         = false;
    float lastRefF           = 0.0f;
    float iAccum             = 0.0f;
    float filteredEncSteps   = 0.0f;
    float filteredRecSteps   = 0.0f;
    bool  filterInit         = false;
    // v13: arm-as encoder state (PID-bron)
    float armLastDeg         = 0.0f;
    float armUnwrappedDeg    = 0.0f;
    long  armEncZeroOffset   = 0;
} mc[3];

uint32_t homeStableStart  = 0;
bool     homeInTolerance  = false;

File     recFile;
uint32_t recStartMs       = 0;
uint32_t lastSampleMs     = 0;
bool     errorFlag        = false;

// ─────────────────────────── HELPERS ────────────────────────
static inline float rawToDeg(uint16_t raw12) { return raw12 * (360.0f / 4096.0f); }

// angleDiff: kortste hoek van 'from' naar 'to', bereik ±180°
static inline float angleDiff(float from, float to) {
    float d = to - from;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// wrapTo180: wikkel willekeurige hoek naar ±180° bereik
static inline float wrapTo180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static inline long degToSteps(float deg) {
    return lround((deg / 360.0f) * (float)STEPS_PER_REV);
}
// v13: arm-graden → stappen (22400 stappen per armomwenteling)
static inline long armDegToSteps(float deg) {
    return lround((deg / 360.0f) * (float)(STEPS_PER_REV) * GEAR_RATIO);
}
static inline float clampF(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// v17: VACTUAL-correctie per motor
// M1 = 200-staps → 51200 µsteps/rev → halve VACTUAL voor zelfde RPM als M2/M3 (102400 µsteps/rev)
// M1 richting omgekeerd (negatief) — v16 homing toonde aanhoudende fout op M1:
//   error bleef −10..−16° bij spd=−16000 → motor draaide verkeerde kant op.
// M2, M3 = 400-staps → geen correctie nodig
// 2026-06-09 meting via driver-test firmware + AS5600 encoder bevestigt:
//   M1 = 200-step (direction omgekeerd)
//   M2 = 200-step (normaal direction)        — v18.1 comment was correct
//   M3 = 400-step (normaal direction)        — v18.1 comment was FOUT, dit is 400-step
// STEPS_PER_REV = 102400 (= 256 µstep × 400 full-step) gaat uit van 400-step.
// CORR magnitude = 0.5 → halve VACTUAL = compensation voor 200-step motor.
// CORR magnitude = 1.0 → volle VACTUAL = correct voor 400-step motor.
static const float MOTOR_VACTUAL_CORR[3] = { -0.5f, 0.5f, 1.0f };

// v14: motorsturing via VACTUAL — vervangt steppers[i].setSpeed()
// steps_per_s wordt omgezet naar VACTUAL register waarde
// v16: MOTOR_VACTUAL_CORR compenseert verschil in stapresolutie per motor
void setMotorSpeed(int idx, float steps_per_s) {
    currentSpeed[idx] = steps_per_s;
    // VACTUAL richting is omgekeerd t.o.v. AccelStepper (equivalent aan TMC2209_DIR_INVERT=true)
    int32_t vactual = -(int32_t)(steps_per_s * VACTUAL_SCALE * MOTOR_VACTUAL_CORR[idx]);
    drivers[idx].moveAtVelocity(vactual);
}

bool fellEdge(uint8_t pin) {
    static uint32_t lastChange[64] = {0};
    static uint8_t  lastState[64]  = {0};
    uint8_t  s   = (uint8_t)digitalRead(pin);
    uint32_t now = millis();
    if (s != lastState[pin] && (now - lastChange[pin]) > 30) {
        lastChange[pin] = now; lastState[pin] = s;
        if (s == LOW) return true;
    }
    return false;
}

struct LongPress {
    uint32_t t0 = 0; bool armed = false; bool fired = false;
    bool update(uint8_t pin, uint32_t holdMs) {
        if (digitalRead(pin) == LOW) {
            if (!armed) { armed = true; fired = false; t0 = millis(); }
            if (!fired && (millis() - t0 >= holdMs)) { fired = true; return true; }
        } else { armed = false; fired = false; }
        return false;
    }
    float progress(uint32_t holdMs) {
        if (!armed) return 0.0f;
        float p = (float)(millis() - t0) / (float)holdMs;
        return (p > 1.0f) ? 1.0f : p;
    }
    bool isHolding() { return armed && !fired; }
};

LongPress lpPlay;
LongPress lpDel;

// ─────────────────────────── LED HELPER ─────────────────────
void updateLedForMode() {
    if (errorFlag) { ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0)); return; }
    switch (mode) {
        case IDLE:
            if (!homeIsSet) ledSet(LED_PULSE_SLOW, CRGB(255, 80, 0));
            else            ledSet(LED_PULSE_SLOW, CRGB(80, 140, 255));
            break;
        case RECORDING: ledSet(LED_BLINK_1HZ,  CRGB(220, 0, 0));    break;
        case HOMING:    ledSet(LED_PULSE_FAST,  CRGB(0, 200, 255));  break;
        case PLAYBACK:  ledSet(LED_SOLID,       CRGB(0, 180, 0));    break;
    }
}

// ─────────────────────────── TCA9548A ───────────────────────
bool tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

void tcaClose() {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
}

// ─────────────────────────── AS5600 ─────────────────────────
bool i2cReadAngle12(uint16_t &angle12) {
    // v12: STATUS check removed — all sensors show MD=0 but output valid angles.
    // AS5600 outputs angles even when STATUS bits indicate weak/missing field,
    // as long as the magnet is present and I2C communication succeeds.
    // We rely on I2C success/failure only, not the STATUS register.
    Wire.beginTransmission(AS5600_ADDR); Wire.write(REG_ANGLE_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return false;
    uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
    angle12 = ((uint16_t)(hi & 0x0F) << 8) | lo;
    return true;
}

// Motor-as encoder: delta-unwrapping, gebruikt door PID en homing
bool readEncoderSteps(int idx, long &stepsOut) {
    if (!tcaSelect(TCA_CHANNELS[idx])) return false;
    uint16_t angle12 = 0;
    bool ok = i2cReadAngle12(angle12);
    tcaClose();
    if (!ok) return false;

    float degNow = rawToDeg(angle12);
    float d = degNow - mc[idx].lastDeg;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    mc[idx].unwrappedDeg += d;
    mc[idx].lastDeg = degNow;

    stepsOut = ENC_DIR * degToSteps(mc[idx].unwrappedDeg) - mc[idx].encZeroOffsetSteps;
    return true;
}

// Motor-as raw hoek (voor home opslaan en Vernier)
bool readRawAngle(int idx, uint16_t &raw) {
    if (!tcaSelect(TCA_CHANNELS[idx])) return false;
    uint16_t angle12 = 0;
    bool ok = i2cReadAngle12(angle12);
    tcaClose();
    if (!ok) return false;
    raw = angle12;
    return true;
}

// v11: Arm-as raw hoek (voor home opslaan en Vernier)
bool readRawArmAngle(int idx, uint16_t &raw) {
    if (!tcaSelect(TCA_ARM_CHANNELS[idx])) return false;
    uint16_t angle12 = 0;
    bool ok = i2cReadAngle12(angle12);
    tcaClose();
    if (!ok) return false;
    raw = angle12;
    return true;
}

bool zeroEncoder(int idx) {
    uint16_t raw = 0;
    if (!readRawAngle(idx, raw)) return false;
    mc[idx].lastDeg            = rawToDeg(raw);
    mc[idx].unwrappedDeg       = 0.0f;
    mc[idx].encZeroOffsetSteps = 0;
    mc[idx].lastError          = 0;
    currentSpeed[idx]       = 0.0f;
    mc[idx].inDeadBand         = false;
    mc[idx].lastRefF           = 0.0f;
    mc[idx].iAccum             = 0.0f;
    return true;
}

// v13: arm-as encoder lezen als PID-bron
// Output in dezelfde stappen-eenheid als readEncoderSteps:
//   358400 stappen = 1 armomwenteling (= STEPS_PER_REV × GEAR_RATIO)
bool readArmEncoderSteps(int idx, long &stepsOut) {
    if (!tcaSelect(TCA_ARM_CHANNELS[idx])) return false;
    uint16_t angle12 = 0;
    bool ok = i2cReadAngle12(angle12);
    tcaClose();
    if (!ok) return false;

    float degNow = rawToDeg(angle12);
    float d = degNow - mc[idx].armLastDeg;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    mc[idx].armUnwrappedDeg += d;
    mc[idx].armLastDeg = degNow;

    stepsOut = ARM_ENC_DIR * armDegToSteps(mc[idx].armUnwrappedDeg) - mc[idx].armEncZeroOffset;
    return true;
}

// v13: arm-encoder nulstellen na succesvolle homing
bool zeroArmEncoder(int idx) {
    uint16_t raw = 0;
    if (!readRawArmAngle(idx, raw)) return false;
    mc[idx].armLastDeg       = rawToDeg(raw);
    mc[idx].armUnwrappedDeg  = 0.0f;
    mc[idx].armEncZeroOffset = 0;
    return true;
}

// ─────────────────────────── NVS HOME ───────────────────────
void loadHomeFromNVS() {
    prefs.begin(NVS_NS, true);
    homeIsSet = prefs.getBool(NVS_KEY_SET, false);
    char key[12];
    for (int i = 0; i < 3; i++) {
        snprintf(key, sizeof(key), "homeRaw%d", i);
        homeAngleRaw[i] = prefs.getUShort(key, 0);
        snprintf(key, sizeof(key), "homeArm%d", i);      // v11: arm-as
        homeArmRaw[i]   = prefs.getUShort(key, 0);
    }
    prefs.end();
    if (homeIsSet) {
        Serial.println("Home geladen:");
        for (int i = 0; i < 3; i++)
            Serial.printf("  M%d motor=%.1f°  arm=%.1f°\n",
                          i+1, rawToDeg(homeAngleRaw[i]), rawToDeg(homeArmRaw[i]));
    } else {
        Serial.println("Geen home. Houd BTN_PLAY lang in om home op te slaan.");
    }
}

// v11: saveHomeToNVS slaat nu ook arm-as raw waarden op
void saveHomeToNVS(uint16_t rawMotor[3], uint16_t rawArm[3]) {
    prefs.begin(NVS_NS, false);
    char key[12];
    for (int i = 0; i < 3; i++) {
        snprintf(key, sizeof(key), "homeRaw%d", i);
        prefs.putUShort(key, rawMotor[i]);
        homeAngleRaw[i] = rawMotor[i];
        snprintf(key, sizeof(key), "homeArm%d", i);
        prefs.putUShort(key, rawArm[i]);
        homeArmRaw[i] = rawArm[i];
    }
    prefs.putBool(NVS_KEY_SET, true);
    prefs.end();
    homeIsSet = true;
    Serial.println("Home OPGESLAGEN (alle 3 assen):");
    for (int i = 0; i < 3; i++)
        Serial.printf("  M%d motor=%.1f°  arm=%.1f°\n",
                      i+1, rawToDeg(homeAngleRaw[i]), rawToDeg(homeArmRaw[i]));
}

// ─────────────────────────── VERNIER (v11) ──────────────────
// Bepaal absolute arm-positie t.o.v. home via Vernier-methode.
// Leest beide sensors, test twee revolutie-hypothesen, kiest
// degene waarvan de verwachte motorsensorwaarde het best
// overeenkomt met de gemeten waarde.
//
// Vereiste: gear ratio = 7/2 (3.5:1). Dan geldt:
//   Rev 0 (0–360° arm):   exp. Δθ_m = Δθ_a × 3.5          (mod ±180°)
//   Rev 1 (360–720° arm): exp. Δθ_m = Δθ_a × 3.5 + 180°   (mod ±180°)
//   Rev −1 (−360–0° arm): exp. Δθ_m = Δθ_a × 3.5 − 180°   (mod ±180°)
//
// De drie hypothesen zijn altijd 180° van elkaar gescheiden op
// de motorsensor, waardoor discriminatie betrouwbaar is zolang
// sensorruis < 90°. In de praktijk is ruis < 1°.
bool initFromVernier() {
    if (!homeIsSet) {
        Serial.println("Vernier: geen home opgeslagen, overgeslagen.");
        return false;
    }

    Serial.println("Vernier absolute positiebepaling:");
    bool allOk = true;

    for (int i = 0; i < 3; i++) {
        uint16_t armRaw = 0, motorRaw = 0;
        if (!readRawArmAngle(i, armRaw)) {
            Serial.printf("  M%d: arm-sensor niet leesbaar!\n", i+1);
            allOk = false; continue;
        }
        if (!readRawAngle(i, motorRaw)) {
            Serial.printf("  M%d: motor-sensor niet leesbaar!\n", i+1);
            allOk = false; continue;
        }

        float homeArmDeg   = rawToDeg(homeArmRaw[i]);
        float homeMotorDeg = rawToDeg(homeAngleRaw[i]);
        float currArmDeg   = rawToDeg(armRaw);
        float currMotorDeg = rawToDeg(motorRaw);

        // Verplaatsing van home in elk sensor-frame (beide ±180°)
        float diffA = ARM_ENC_DIR * angleDiff(homeArmDeg, currArmDeg);  // v12c: arm sensor richting omgekeerd
        float diffM = angleDiff(homeMotorDeg, currMotorDeg);  // motor-sensor delta

        // Drie hypothesen voor de werkelijke arm-verplaatsing:
        //   H0:  diffA +   0° (eerste omwenteling, zelfde richting)
        //   H1:  diffA + 360° (één omwenteling meer)
        //   H-1: diffA − 360° (één omwenteling minder)
        float candidates[3] = { diffA, diffA + 360.0f, diffA - 360.0f };

        float bestDelta     = 1e9f;
        float bestArmFromHome = diffA;
        int   bestRev       = 0;

        for (int k = -1; k <= 1; k++) {
            float armHyp     = diffA + (float)k * 360.0f;
            float expDiffM   = wrapTo180(armHyp * GEAR_RATIO);
            float delta      = fabsf(angleDiff(expDiffM, diffM));
            if (delta < bestDelta) {
                bestDelta       = delta;
                bestArmFromHome = armHyp;
                bestRev         = k;
            }
        }

        // Betrouwbaarheidscheck: beste match moet duidelijk beter zijn
        // dan de andere twee hypothesen (minimaal drempelwaarde)
        if (bestDelta > VERNIER_MATCH_THRESHOLD_DEG) {
            Serial.printf("  M%d: WAARSCHUWING Vernier match zwak (delta=%.1f°) — "
                          "fallback naar raw-angle methode\n", i+1, bestDelta);
            // Fallback: gebruik alleen arm-sensor (±180° arm limiet)
            bestArmFromHome = diffA;
        }

        // Zet motor unwrappedDeg = arm_verplaatsing × gear ratio
        mc[i].unwrappedDeg  = bestArmFromHome * GEAR_RATIO;
        mc[i].lastDeg       = currMotorDeg;   // delta-tracker basis (motor-as)
        encValidFromHome[i] = true;

        // v13: arm-encoder state initialiseren vanuit Vernier resultaat.
        // armUnwrappedDeg moet ALTIJD raw sensor-coördinaten bevatten (zonder ARM_ENC_DIR).
        // De sensor draait tegengesteld aan de arm (ARM_ENC_DIR=-1), dus:
        //   fysieke arm op +30° → sensor-raw op -30° → armUnwrappedDeg = -30°
        // readArmEncoderSteps past ARM_ENC_DIR toe bij output → stepsOut correct positief.
        mc[i].armUnwrappedDeg  = ARM_ENC_DIR * bestArmFromHome;  // fysiek → sensor-coords
        mc[i].armLastDeg       = currArmDeg;                      // raw delta-basis
        mc[i].armEncZeroOffset = 0;

        float armDeg = bestArmFromHome;
        Serial.printf("  M%d: arm=%.1f° (rev%+d)  motor=%.1f°  "
                      "match_delta=%.1f°  %s\n",
                      i+1, armDeg, bestRev,
                      mc[i].unwrappedDeg,
                      bestDelta,
                      bestDelta <= VERNIER_MATCH_THRESHOLD_DEG ? "✓" : "! zwak");
    }

    return allOk;
}

// ─────────────────────────── SD ─────────────────────────────
bool initSD() {
    pinMode(SD_DET, INPUT_PULLDOWN);
    if (!digitalRead(SD_DET)) { Serial.println("No SD card."); return false; }
    if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3)) return false;
    if (!SD_MMC.begin("/sdcard", false)) return false;
    Serial.printf("SD OK, %llu MB\n", SD_MMC.cardSize() / (1024ULL*1024ULL));
    return true;
}

// ─────────────────────────── HOMING ─────────────────────────
bool startHoming() {
    if (!homeIsSet) { Serial.println("No home set!"); return false; }

    homingUseMultiTurn = encValidFromHome[0] && encValidFromHome[1] && encValidFromHome[2];

    // BUG FIX (2026-06-09): defensive re-assert TMC config — library state-drift
    for (int i = 0; i < 3; i++) {
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
    }

    for (int i = 0; i < 3; i++) {
        drivers[i].enable();
                setMotorSpeed(i, 0.0f);

        mc[i].lastError      = 0;
        currentSpeed[i]   = 0.0f;
        mc[i].inDeadBand     = false;
        mc[i].lastRefF       = 0.0f;
        mc[i].iAccum         = 0.0f;

        if (homingUseMultiTurn) {
            long armSteps = 0;
            if (!readArmEncoderSteps(i, armSteps)) {
                for (int j = 0; j <= i; j++) drivers[j].disable();
                return false;
            }
            float armPos  = (float)armSteps / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
            float armDiff = -armPos;
            Serial.printf("HOMING M%d: arm_pos=%.1f°  arm_diff=%.1f°  (%ld stappen) [arm-enc]\n",
                          i+1, armPos, armDiff, -armSteps);
        } else {
            // Fallback: raw-angle (alleen bij allereerste home-instelling ooit)
            if (!zeroEncoder(i)) {
                for (int j = 0; j <= i; j++) drivers[j].disable();
                return false;
            }
            if (!zeroArmEncoder(i)) {   // v13: arm-encoder ook nulstellen in fallback
                for (int j = 0; j <= i; j++) drivers[j].disable();
                return false;
            }
            uint16_t raw = 0;
            if (!readRawAngle(i, raw)) {
                for (int j = 0; j <= i; j++) drivers[j].disable();
                return false;
            }
            float diff    = angleDiff(rawToDeg(raw), rawToDeg(homeAngleRaw[i]));
            float armDiff = diff / GEAR_RATIO;
            // Arm-encoder: zeroArmEncoder() heeft armUnwrappedDeg=0 en armLastDeg=sensor_raw gezet.
            // De offset corrigeert de uitgang van readArmEncoderSteps zodat die de huidige
            // arm-afstand tot home retourneert: stepsOut = 0 - offset = armDegToSteps(armDiff).
            // armUnwrappedDeg NIET overschrijven — moet 0 blijven (sensor-coördinaten).
            mc[i].armEncZeroOffset = ARM_ENC_DIR * armDegToSteps(armDiff);  // = -armDegToSteps(armDiff)
            if (fabsf(diff) > 150.0f)
                Serial.printf("  WAARSCHUWING M%d: motorhoek verschil %.1f° > ±51.4° arm limiet!\n",
                              i+1, diff);
            Serial.printf("HOMING M%d: motor_diff=%.1f°  arm_diff=%.1f°  (%ld stappen)\n",
                          i+1, diff, armDiff, degToSteps(diff));
        }
    }
    homeStableStart  = 0;
    homeInTolerance  = false;
    return true;
}

void handleHoming() {
    static uint32_t lastCtrl  = 0;
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastCtrl < CTRL_MS) return;
    lastCtrl = now;

    bool  allInTol     = true;
    float armErrArr[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 3; i++) {
        long error;

        if (homingUseMultiTurn) {
            long armSteps = 0;
            if (!readArmEncoderSteps(i, armSteps)) { setMotorSpeed(i, 0.0f); return; }
            error        = -armSteps;
            armErrArr[i] = -(float)armSteps / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
        } else {
            // Fallback (eerste keer, nooit home opgeslagen): ook arm-encoder gebruiken
            long armSteps = 0;
            if (!readArmEncoderSteps(i, armSteps)) { setMotorSpeed(i, 0.0f); return; }
            error        = -armSteps;
            armErrArr[i] = -(float)armSteps / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
        }

        long  dErr  = error - mc[i].lastError;
        mc[i].lastError = error;
        if (labs(error) > HOME_TOLERANCE_STEPS) allInTol = false;

        float speedCmd = KP_HOME * (float)error + KD_HOME * (float)dErr;
        speedCmd = clampF(speedCmd, -MAX_HOME_SPEED, +MAX_HOME_SPEED);
        if      (error > 0 && speedCmd > 0 && speedCmd < MIN_SPEED)  speedCmd =  MIN_SPEED;
        else if (error < 0 && speedCmd < 0 && speedCmd > -MIN_SPEED) speedCmd = -MIN_SPEED;

        float accelLimit = ACCEL_STEPS_S2 * ((float)CTRL_MS / 1000.0f);
        float delta = clampF(speedCmd - currentSpeed[i], -accelLimit, +accelLimit);
        currentSpeed[i] += delta;
        setMotorSpeed(i, currentSpeed[i]);
    }

    if (allInTol) {
        if (!homeInTolerance) { homeInTolerance = true; homeStableStart = now; }
        if (now - homeStableStart >= HOME_STABLE_MS) {
            for (int i = 0; i < 3; i++) {
                setMotorSpeed(i, 0.0f);
                zeroEncoder(i);
                zeroArmEncoder(i);         // v13: arm-encoder ook nulstellen
                encValidFromHome[i] = true;
            }
            Serial.println("HOMING done → starting playback");
            if (startPlayback()) { mode = PLAYBACK; }
            else { for (int i = 0; i < 3; i++) drivers[i].disable(); mode = IDLE; }
            updateLedForMode();
            return;
        }
    } else { homeInTolerance = false; }

    if (now - lastPrint >= PRINT_MS) {
        lastPrint = now;
        for (int i = 0; i < 3; i++) {
            Serial.printf("HOME M%d arm_err=%.1f° spd=%.0f | ", i+1, armErrArr[i], currentSpeed[i]);
        }
        Serial.printf("%s\n", homeInTolerance ? "[STABLE]" : "");
    }
}

// ─────────────────────────── RECORD ─────────────────────────
bool startRecording() {
    // BUG FIX (2026-06-09): defensive re-assert TMC config — library state-drift
    for (int i = 0; i < 3; i++) {
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
    }
    for (int i = 0; i < 3; i++) {
        if (!encValidFromHome[i]) {
            Serial.println("!! Eerst homen (BTN_PLAY kort) voor opname.");
            ledSet(LED_BLINK_5HZ, CRGB(255, 80, 0)); delay(1000); updateLedForMode();
            return false;
        }
        long steps = 0;
        readArmEncoderSteps(i, steps);   // v13: arm-encoder als referentie
        if (labs(steps) > REC_HOME_TOLERANCE_STEPS) {
            float armDeg = (float)steps / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
            Serial.printf("!! M%d niet bij home (%.1f° arm, %ld stappen). Eerst homen.\n",
                          i+1, armDeg, steps);
            ledSet(LED_BLINK_5HZ, CRGB(255, 80, 0)); delay(1000); updateLedForMode();
            return false;
        }
    }

    for (int i = 0; i < 3; i++) {
        drivers[i].disable();
        recStartSteps[i]         = 0;
        mc[i].filteredRecSteps   = 0.0f;
        mc[i].filterInit         = false;
    }
    recStartMs = 0; lastSampleMs = 0;
    if (SD_MMC.exists(MOTION_FILE)) SD_MMC.remove(MOTION_FILE);
    recFile = SD_MMC.open(MOTION_FILE, FILE_WRITE);
    if (!recFile) { Serial.println("Failed to open motion file."); return false; }
    recFile.println("t_ms,steps_1,steps_2,steps_3");
    recFile.flush();
    Serial.println("RECORDING started (arm at home verified).");
    return true;
}

void stopRecording() {
    if (recFile) { recFile.flush(); recFile.close(); recFile = File(); }
    recStartMs = 0; lastSampleMs = 0;
    for (int i = 0; i < 3; i++) drivers[i].disable();
    Serial.println("RECORDING stopped + saved.");
}

void handleRecording() {
    uint32_t now = millis();
    if (now - lastSampleMs < SAMPLE_MS) return;
    lastSampleMs = now;
    if (recStartMs == 0) {
        recStartMs = now;
        for (int i = 0; i < 3; i++) mc[i].filterInit = false;
    }
    uint32_t relT = now - recStartMs;

    long encSteps[3] = {0, 0, 0};
    bool ok = true;
    for (int i = 0; i < 3; i++) {
        if (!readArmEncoderSteps(i, encSteps[i])) { ok = false; break; }  // v13: arm-encoder
        encSteps[i] -= recStartSteps[i];
    }
    if (!ok) { Serial.println("REC: encoder fail."); return; }

    long filteredSteps[3];
    for (int i = 0; i < 3; i++) {
        if (!mc[i].filterInit) {
            mc[i].filteredRecSteps = (float)encSteps[i];
            mc[i].filterInit = true;
        } else {
            mc[i].filteredRecSteps = REC_FILTER_ALPHA * (float)encSteps[i]
                                   + (1.0f - REC_FILTER_ALPHA) * mc[i].filteredRecSteps;
        }
        filteredSteps[i] = lround(mc[i].filteredRecSteps);
    }

    if (recFile)
        recFile.printf("%lu,%ld,%ld,%ld\n",
            (unsigned long)relT, filteredSteps[0], filteredSteps[1], filteredSteps[2]);

    static uint32_t lastFlush = 0;
    if (now - lastFlush > 2000) { lastFlush = now; if (recFile) recFile.flush(); }

    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.printf("REC t=%lu  M1=%.1f°  M2=%.1f°  M3=%.1f°\n",
            (unsigned long)relT,
            (float)filteredSteps[0] / (STEPS_PER_REV * GEAR_RATIO) * 360.0f,
            (float)filteredSteps[1] / (STEPS_PER_REV * GEAR_RATIO) * 360.0f,
            (float)filteredSteps[2] / (STEPS_PER_REV * GEAR_RATIO) * 360.0f);
    }
}

// ─────────────────────────── PLAYBACK ───────────────────────
struct Playback {
    File     f;
    uint32_t startMs     = 0;
    float    virtualMs   = 0.0f;
    float    timeScaleLP = 1.0f;

    uint32_t tPrev=0, t0=0, t1=0, tNext=0;
    long     sPrev[3]={}, s0[3]={}, s1[3]={}, sNext[3]={};

    bool hasSeg      = false;
    bool hasNext     = false;
    bool hasNextNext = false;
    char line[128];

    // === DIAGNOSTIEK v14.4 ===
    long     maxAbsError[3]   = {0,0,0};   // max |refSteps - encFiltered| per as
    float    maxAbsCmdPre[3]  = {0,0,0};   // max |speedCmd voor clamp| per as
    uint32_t clipTicks[3]     = {0,0,0};   // # ticks waar |cmdPre| > MAX_SPEED
    uint32_t totalTicks       = 0;          // totaal aantal handlePlayback-ticks
    float    minTimeScaleSeen = 1.0f;       // laagste timeScaleLP gezien

    void close() {
        if (f) { f.close(); f = File(); }
        startMs = 0; virtualMs = 0.0f; timeScaleLP = 1.0f;
        tPrev=t0=t1=tNext=0;
        for (int i=0;i<3;i++) sPrev[i]=s0[i]=s1[i]=sNext[i]=0;
        hasSeg=hasNext=hasNextNext=false;
        // diagnostiek-reset
        for (int i=0;i<3;i++) { maxAbsError[i]=0; maxAbsCmdPre[i]=0.0f; clipTicks[i]=0; }
        totalTicks = 0; minTimeScaleSeen = 1.0f;
    }
} pb;

bool parseRow(char *buf, uint32_t &tOut, long sOut[3]) {
    char *cr = strchr(buf, '\r'); if (cr) *cr = 0;
    if (buf[0] == 0 || buf[0] == 't') return false;
    char *p = buf;
    tOut = (uint32_t)strtoul(p, &p, 10); if (*p == ',') p++;
    for (int i = 0; i < 3; i++) {
        sOut[i] = (long)strtol(p, &p, 10); if (*p == ',') p++;
    }
    return true;
}

bool readNextPoint(uint32_t &tOut, long sOut[3]) {
    if (!pb.f) return false;
    while (pb.f.available()) {
        size_t n = pb.f.readBytesUntil('\n', pb.line, sizeof(pb.line)-1);
        pb.line[n] = 0;
        if (parseRow(pb.line, tOut, sOut)) return true;
    }
    return false;
}

bool startPlayback() {
    if (!SD_MMC.exists(MOTION_FILE)) { Serial.println("No motion file."); return false; }
    File f = SD_MMC.open(MOTION_FILE, FILE_READ);
    if (!f) { Serial.println("Failed to open motion file."); return false; }

    // BUG FIX (2026-06-09): defensive re-assert TMC config — library state-drift
    for (int i = 0; i < 3; i++) {
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].disableStealthChop();
    }

    for (int i = 0; i < 3; i++) {
                setMotorSpeed(i, 0.0f);
        mc[i].lastError = 0; mc[i].dErrFilt = 0.0f; currentSpeed[i] = 0.0f;
        mc[i].inDeadBand = false; mc[i].lastRefF = 0.0f; mc[i].iAccum = 0.0f;
        mc[i].filterInit = false;
    }

    pb.close(); pb.f = f; pb.startMs = millis(); pb.timeScaleLP = 1.0f;

    uint32_t ta, tb, tc; long sa[3], sb[3], sc[3];
    if (!readNextPoint(ta, sa)) { Serial.println("Motion file empty."); pb.close(); return false; }

    if (!readNextPoint(tb, sb)) {
        pb.tPrev=ta; for(int i=0;i<3;i++) pb.sPrev[i]=sa[i];
        pb.t0=ta; for(int i=0;i<3;i++) pb.s0[i]=sa[i];
        pb.t1=ta+1; for(int i=0;i<3;i++) pb.s1[i]=sa[i];
        pb.hasSeg=true; pb.hasNext=false;
    } else {
        pb.tPrev=ta; for(int i=0;i<3;i++) pb.sPrev[i]=sa[i];
        pb.t0=ta;    for(int i=0;i<3;i++) pb.s0[i]=sa[i];
        pb.t1=tb;    for(int i=0;i<3;i++) pb.s1[i]=sb[i];
        pb.hasSeg=true; pb.hasNext=true;
        if (readNextPoint(tc, sc)) {
            pb.tNext=tc; for(int i=0;i<3;i++) pb.sNext[i]=sc[i]; pb.hasNextNext=true;
        } else {
            pb.tNext=tb; for(int i=0;i<3;i++) pb.sNext[i]=sb[i]; pb.hasNextNext=false;
        }
    }

    Serial.println("PLAYBACK started (3-axis Hermite, smoother v14.3).");
    return true;
}

void stopPlayback() {
    pb.close();
    for (int i = 0; i < 3; i++) {
        currentSpeed[i] = 0.0f;
        setMotorSpeed(i, 0.0f);
        drivers[i].disable();
    }
    Serial.println("PLAYBACK stopped.");
}

float currentRefSteps(int idx, uint32_t elapsed) {
    if (!pb.hasSeg) return 0.0f;

    float dt = (float)((pb.t1 > pb.t0) ? (pb.t1 - pb.t0) : 1);
    float x  = (float)((elapsed >= pb.t0) ? (elapsed - pb.t0) : 0);
    if (x > dt) x = dt;
    float u = x / dt;

    float u2=u*u, u3=u2*u;
    float h00= 2*u3-3*u2+1, h10=u3-2*u2+u, h01=-2*u3+3*u2, h11=u3-u2;

    float dtPrev = (float)((pb.t0 > pb.tPrev && pb.t0 != pb.tPrev) ? (pb.t0 - pb.tPrev) : dt);
    float dtNext = (float)((pb.tNext > pb.t1 && pb.hasNextNext)    ? (pb.tNext - pb.t1)  : dt);

    float m0 = ((float)(pb.s1[idx] - pb.sPrev[idx]) / (dtPrev + dt)) * dt;
    float m1 = ((float)(pb.sNext[idx] - pb.s0[idx]) / (dt + dtNext)) * dt;

    float maxTangent = MAX_SPEED_STEPS_S * (dt / 1000.0f);
    m0 = clampF(m0, -maxTangent, +maxTangent);
    m1 = clampF(m1, -maxTangent, +maxTangent);

    return h00*(float)pb.s0[idx] + h10*m0 + h01*(float)pb.s1[idx] + h11*m1;
}

void handlePlayback() {
    static uint32_t lastCtrl  = 0;
    static uint32_t lastPrint = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastCtrl < CTRL_MS) return;
    lastCtrl = nowMs;

    uint32_t elapsed = (uint32_t)pb.virtualMs;
    while (pb.hasNext && elapsed >= pb.t1) {
        pb.tPrev = pb.t0; pb.t0 = pb.t1; pb.t1 = pb.tNext;
        for (int i = 0; i < 3; i++) {
            pb.sPrev[i]=pb.s0[i]; pb.s0[i]=pb.s1[i]; pb.s1[i]=pb.sNext[i];
        }
        uint32_t tn; long sn[3];
        if (readNextPoint(tn, sn)) {
            pb.tNext=tn; for(int i=0;i<3;i++) pb.sNext[i]=sn[i]; pb.hasNextNext=true;
        } else {
            pb.tNext=pb.t1; for(int i=0;i<3;i++) pb.sNext[i]=pb.s1[i];
            pb.hasNextNext=false; pb.hasNext=false;
        }
    }

    float minTimeScale = 1.0f;

    // === DIAGNOSTIEK v14.4 — per-tick buffers, gebruikt in print/CSV onder ===
    float cmdPreBuf[3] = {0,0,0};
    bool  clippedBuf[3] = {false,false,false};
    long  errorBuf[3]  = {0,0,0};
    float refVelBuf[3] = {0,0,0};

    for (int i = 0; i < 3; i++) {
        long encSteps = 0;
        if (!readArmEncoderSteps(i, encSteps)) {   // v13: arm-encoder als PID-bron
            currentSpeed[i] = 0.0f; setMotorSpeed(i, 0.0f);
            errorFlag = true; updateLedForMode();
            return;
        }
        errorFlag = false;

        if (!mc[i].filterInit) {
            mc[i].filteredEncSteps = (float)encSteps;
            mc[i].filterInit = true;
        } else {
            mc[i].filteredEncSteps = PID_FILTER_ALPHA * (float)encSteps
                                   + (1.0f - PID_FILTER_ALPHA) * mc[i].filteredEncSteps;
        }
        long encFiltered = lround(mc[i].filteredEncSteps);

        float refF     = currentRefSteps(i, elapsed);
        long  refSteps = lround(refF);

        // v19: soft endstop clamp — als arm voorbij limit, hou positie vast
        // BUG FIX (2026-06-09): naam-agnostisch via fmin/fmax. Origineel v19 brak
        // wanneer min > max in NVS (gebeurt door ARM_ENC_DIR=-1: "links" is sensor-grote,
        // "rechts" sensor-kleinere). Dan triggerde clamp altijd → arm bewoog niet.
        {
            float curArmDeg = mc[i].armUnwrappedDeg;
            bool hasMin = !isnan(lim_min_deg[i]);
            bool hasMax = !isnan(lim_max_deg[i]);
            if (hasMin && hasMax) {
                float lo = fminf(lim_min_deg[i], lim_max_deg[i]);
                float hi = fmaxf(lim_min_deg[i], lim_max_deg[i]);
                if (curArmDeg >= hi || curArmDeg <= lo) refSteps = encFiltered;
            } else if (hasMax && curArmDeg >= lim_max_deg[i]) {
                refSteps = encFiltered;
            } else if (hasMin && curArmDeg <= lim_min_deg[i]) {
                refSteps = encFiltered;
            }
        }

        long  error    = refSteps - encFiltered;

        // === DIAGNOSTIEK v14.4 ===
        errorBuf[i] = error;
        if ((unsigned long)labs(error) > (unsigned long)pb.maxAbsError[i])
            pb.maxAbsError[i] = labs(error);

        float refVel = (refF - mc[i].lastRefF) * (1000.0f / (float)CTRL_MS);
        mc[i].lastRefF = refF;
        refVelBuf[i] = refVel;

        float velFrac  = clampF(fmaxf(fabsf(refVel), fabsf(currentSpeed[i])) / MAX_SPEED_STEPS_S, 0.0f, 1.0f);
        float scaleMax = (float)PEAK_FOLLOW_ERROR_STEPS + (float)(MAX_FOLLOW_ERROR_STEPS - PEAK_FOLLOW_ERROR_STEPS) * velFrac;

        float errorRatio = clampF((float)labs(error) / scaleMax, 0.0f, 1.0f);
        float timeScale  = 1.0f - errorRatio * errorRatio;
        if (timeScale < minTimeScale) minTimeScale = timeScale;

        float kp = KP_SPEED[i] * (1.0f + (KP_PEAK_MAX - 1.0f) * (1.0f - velFrac));
        float refVelScaled = refVel * timeScale;
        long  dErr = error - mc[i].lastError; mc[i].lastError = error;
        mc[i].dErrFilt = DERR_FILTER_ALPHA * (float)dErr + (1.0f - DERR_FILTER_ALPHA) * mc[i].dErrFilt;

        if (fabsf(refVelScaled) > DB_VEL_THRESHOLD) {
            mc[i].inDeadBand = false;
        } else {
            if ( mc[i].inDeadBand && labs(error) > DB_ON_STEPS)  mc[i].inDeadBand = false;
            if (!mc[i].inDeadBand && labs(error) < DB_OFF_STEPS) mc[i].inDeadBand = true;
        }

        float speedCmd = 0.0f;
        if (!mc[i].inDeadBand) {
            speedCmd = refVelScaled + kp * (float)error + KD_SPEED[i] * mc[i].dErrFilt;
            // === DIAGNOSTIEK v14.4 — meet voor clamp ===
            cmdPreBuf[i]  = speedCmd;
            clippedBuf[i] = (fabsf(speedCmd) > MAX_SPEED_STEPS_S);
            if (fabsf(speedCmd) > pb.maxAbsCmdPre[i]) pb.maxAbsCmdPre[i] = fabsf(speedCmd);
            if (clippedBuf[i]) pb.clipTicks[i]++;
            // === einde meting ===
            speedCmd = clampF(speedCmd, -MAX_SPEED_STEPS_S, +MAX_SPEED_STEPS_S);
            if (labs(error) < 20) {
                mc[i].iAccum += KI_SPEED[i] * (float)error * (CTRL_MS / 1000.0f);
                mc[i].iAccum  = clampF(mc[i].iAccum, -200.0f, 200.0f);
            } else { mc[i].iAccum *= 0.95f; }
            speedCmd += mc[i].iAccum;
            float errFrac = fminf(1.0f, fabsf((float)error) / 10.0f);
            float minSpeedNow = MIN_SPEED * errFrac;
            if      (error > 0 && speedCmd > 0 && speedCmd < minSpeedNow)  speedCmd =  minSpeedNow;
            else if (error < 0 && speedCmd < 0 && speedCmd > -minSpeedNow) speedCmd = -minSpeedNow;
        }

        float accelLimit = ACCEL_STEPS_S2 * ((float)CTRL_MS / 1000.0f);
        float decelLimit = accelLimit * DECEL_MULTIPLIER;
        bool  braking    = (fabsf(speedCmd) < fabsf(currentSpeed[i]));
        float delta      = clampF(speedCmd - currentSpeed[i],
                                  -(braking ? decelLimit : accelLimit),
                                  +(braking ? decelLimit : accelLimit));
        currentSpeed[i] += delta;
        setMotorSpeed(i, currentSpeed[i]);
    }

    float targetTimeScale = clampF(minTimeScale, MIN_TIME_SCALE, 1.0f);
    pb.timeScaleLP = TS_FILTER_ALPHA * targetTimeScale + (1.0f - TS_FILTER_ALPHA) * pb.timeScaleLP;
    pb.virtualMs += (float)CTRL_MS * pb.timeScaleLP * PLAYBACK_SPEED;    // v14.6: globale trage playback

    // === DIAGNOSTIEK v14.4 ===
    pb.totalTicks++;
    if (pb.timeScaleLP < pb.minTimeScaleSeen) pb.minTimeScaleSeen = pb.timeScaleLP;

#if DENSE_CSV_LOG
    // 200Hz CSV-stream voor latere plot. Format vast — eerste regel komt uit setup-banner of negeer.
    // PBCSV,t_ms,scale, [per as: ref,enc,err,refVel,cmdPre,spd]×3
    Serial.printf("PBCSV,%lu,%.3f", (unsigned long)elapsed, pb.timeScaleLP);
    for (int i = 0; i < 3; i++) {
        float refF_csv = currentRefSteps(i, elapsed);
        Serial.printf(",%ld,%ld,%ld,%.0f,%.0f,%.0f",
            lround(refF_csv),
            lround(mc[i].filteredEncSteps),
            errorBuf[i],
            refVelBuf[i],
            cmdPreBuf[i],
            currentSpeed[i]);
    }
    Serial.println();
#endif

    if (nowMs - lastPrint >= PRINT_MS) {
        lastPrint = nowMs;
        Serial.printf("PB t=%lu sc=%.2f", (unsigned long)elapsed, pb.timeScaleLP);
        for (int i = 0; i < 3; i++) {
            float refF = currentRefSteps(i, elapsed);
            float encDeg = mc[i].filteredEncSteps / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
            float refDeg = (float)lround(refF) / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
            float errDeg = refDeg - encDeg;
            // === DIAGNOSTIEK v14.4: clip-flag + cmdPre als % MAX_SPEED ===
            float cmdFrac = cmdPreBuf[i] / MAX_SPEED_STEPS_S * 100.0f;
            char  clipMark = clippedBuf[i] ? '!' : ' ';
            Serial.printf("  M%d ref=%.1f° enc=%.1f° err=%.2f° spd=%.0f cmd=%+.0f%%%c",
                i+1,
                refDeg,
                encDeg,
                errDeg,
                currentSpeed[i],
                cmdFrac,
                clipMark);
        }
        Serial.println();
    }

    bool allDone = !pb.hasNext && elapsed >= pb.t0;
    if (allDone) {
        // Drempel: 512 stappen ≈ 0.36° arm bij 51200 steps/omw
        static uint32_t settleStartMs = 0;
        bool allSettled = true;
        for (int i = 0; i < 3; i++) {
            long encSteps = 0; readArmEncoderSteps(i, encSteps);   // v13
            long refSteps = lround(currentRefSteps(i, elapsed));
            if (labs(refSteps - encSteps) >= 512) allSettled = false;
        }
        // Timeout: forceer stop na 2 seconden ook als niet volledig gesettled
        if (settleStartMs == 0) settleStartMs = nowMs;
        if (allSettled || (nowMs - settleStartMs > 2000)) {
            settleStartMs = 0;
            // === DIAGNOSTIEK v14.4 — eindsamenvatting ===
            Serial.println();
            Serial.println("=== PLAYBACK DIAGNOSTIEK v14.4 ===");
            Serial.printf("Totaal ticks: %lu  (looptijd ~%.1fs)\n",
                (unsigned long)pb.totalTicks,
                (float)pb.totalTicks * (float)CTRL_MS / 1000.0f);
            Serial.printf("Min timeScaleLP gezien: %.3f  (1.000 = nooit vertraagd)\n",
                pb.minTimeScaleSeen);
            for (int i = 0; i < 3; i++) {
                float maxErrDeg = (float)pb.maxAbsError[i] / (STEPS_PER_REV * GEAR_RATIO) * 360.0f;
                float maxCmdPct = pb.maxAbsCmdPre[i] / MAX_SPEED_STEPS_S * 100.0f;
                float clipPct   = pb.totalTicks ? (100.0f * (float)pb.clipTicks[i] / (float)pb.totalTicks) : 0.0f;
                Serial.printf("M%d: maxErr=%ld steps (%.2f° arm)  maxCmdPre=%.0f (%.0f%% MAX)  clipped=%lu/%lu ticks (%.1f%%)\n",
                    i+1,
                    pb.maxAbsError[i],
                    maxErrDeg,
                    pb.maxAbsCmdPre[i],
                    maxCmdPct,
                    (unsigned long)pb.clipTicks[i],
                    (unsigned long)pb.totalTicks,
                    clipPct);
            }
            Serial.println("Interpretatie:");
            Serial.println("  - clipping >5% op een of meer assen → H1 (speed-clipping) waarschijnlijk");
            Serial.println("  - clipping ~0% maar maxErr groot     → H2 (FF-mismatch) waarschijnlijk");
            Serial.println("  - één as duidelijk slechter dan rest  → H3 (as-asynchroniteit) waarschijnlijk");
            Serial.println("===================================");
            stopPlayback(); mode = IDLE; updateLedForMode();
        }
    }
}

// ─────────────────────────── DELETE ─────────────────────────
void deleteMotionFile() {
    if (SD_MMC.exists(MOTION_FILE))
        Serial.println(SD_MMC.remove(MOTION_FILE) ? "File deleted." : "Delete failed.");
    else
        Serial.println("No motion file to delete.");
}

// ─────────────────────────── v19: SOFT ENDSTOPS ────────────
// Per-joint min/max in arm-degrees (NaN = niet ingesteld).
// In handlePlayback wordt refSteps geklampt: bij overschrijding houdt
// de PID de arm op huidige positie i.p.v. door te duwen.
// Arrays zijn boven gedeclareerd zodat handlePlayback ze kan zien.

// 2026-06-09: per-joint run-current opslaan/laden in NVS — niet langer alleen boot default.
static int currentRunPct[3] = { TMC_RUN_CURRENT, TMC_RUN_CURRENT, TMC_RUN_CURRENT };

static void saveCurrentsToNVS() {
    prefs.begin(NVS_NS, false);
    char k[16];
    for (int i = 0; i < 3; i++) {
        snprintf(k, sizeof(k), "cur%d", i); prefs.putInt(k, currentRunPct[i]);
    }
    prefs.end();
}

static void loadCurrentsFromNVS() {
    prefs.begin(NVS_NS, true);
    char k[16];
    for (int i = 0; i < 3; i++) {
        snprintf(k, sizeof(k), "cur%d", i);
        currentRunPct[i] = prefs.getInt(k, TMC_RUN_CURRENT);
    }
    prefs.end();
}

static void saveLimitsToNVS() {
    prefs.begin(NVS_NS, false);
    char k[16];
    for (int i = 0; i < 3; i++) {
        snprintf(k, sizeof(k), "lmn%d", i); prefs.putFloat(k, lim_min_deg[i]);
        snprintf(k, sizeof(k), "lmx%d", i); prefs.putFloat(k, lim_max_deg[i]);
    }
    prefs.end();
}

static void loadLimitsFromNVS() {
    prefs.begin(NVS_NS, true);
    char k[16];
    for (int i = 0; i < 3; i++) {
        snprintf(k, sizeof(k), "lmn%d", i); lim_min_deg[i] = prefs.getFloat(k, NAN);
        snprintf(k, sizeof(k), "lmx%d", i); lim_max_deg[i] = prefs.getFloat(k, NAN);
    }
    prefs.end();
}

// ─────────────────────────── v19: SERIAL CMD PARSER ────────
// Line-based ASCII parser voor host commando's. Niet-blocking;
// een onvolledige regel wacht tot volgende loop()-iteratie.
// Antwoorden: "<OK ..." of "<ERR ..." — PB-stream blijft intact.

static String cmdLineBuffer = "";

static void cmdReply(const char* prefix, const char* msg) {
    Serial.print(prefix); Serial.println(msg);
}

static void cmdReplyf(const char* prefix, const char* fmt, ...) {
    char buf[200];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(prefix); Serial.println(buf);
}

static void cmdHelp() {
    Serial.println("<OK help");
    Serial.println("  >HELP                       dit overzicht");
    Serial.println("  >STATUS                     mode/home/motion/PID");
    Serial.println("  >MODE IDLE|HOMING|PLAYBACK|RECORDING");
    Serial.println("  >TUNE KP|KI|KD <i> <v>      live PID (i=0..2)");
    Serial.println("  >CURRENT <i> <pct>          TMC run current 0..100% (i=0..2)");
    Serial.println("  >HOME                       home opslaan (= lang BTN_PLAY)");
    Serial.println("  >DEL                        verwijder motion file");
    Serial.println("  >LIMITSET <i> MIN|MAX       leg endstop vast op huidige pos");
    Serial.println("  >LIMITS                     toon endstops + huidige posities");
    Serial.println("  >LIMITSAVE                  NVS write soft endstops");
    Serial.println("  >CURRENTSAVE                NVS write per-joint run-current");
    Serial.println("  >LIMITCLR <i>               wis endstops voor joint i");
    Serial.println("  >ENCRAW                     raw AS5600 hoeken (motor+arm channels) — debug");
    Serial.println("  >TMCSTATUS                  TMC2209 DRV_STATUS per motor (temp/ot/cs/sg)");
}

static void cmdStatus() {
    const char* modeStr = "?";
    switch (mode) {
        case IDLE:      modeStr = "IDLE"; break;
        case RECORDING: modeStr = "RECORDING"; break;
        case HOMING:    modeStr = "HOMING"; break;
        case PLAYBACK:  modeStr = "PLAYBACK"; break;
    }
    cmdReplyf("<OK ", "status mode=%s home=%d motion=%d "
                      "kp=[%.2f,%.2f,%.2f] ki=[%.3f,%.3f,%.3f] kd=[%.2f,%.2f,%.2f]",
              modeStr, homeIsSet ? 1 : 0,
              SD_MMC.exists(MOTION_FILE) ? 1 : 0,
              KP_SPEED[0], KP_SPEED[1], KP_SPEED[2],
              KI_SPEED[0], KI_SPEED[1], KI_SPEED[2],
              KD_SPEED[0], KD_SPEED[1], KD_SPEED[2]);
}

static void cmdGoIdle() {
    if (mode == RECORDING) { stopRecording(); }
    if (mode == PLAYBACK)  { stopPlayback();  }
    if (mode == HOMING) {
        for (int i = 0; i < 3; i++) { setMotorSpeed(i, 0.0f); drivers[i].disable(); }
    }
    mode = IDLE;
    updateLedForMode();
    cmdReply("<OK ", "mode=IDLE");
}

static void cmdMode(const String& target) {
    if (target == "IDLE") {
        cmdGoIdle();
    } else if (target == "RECORDING") {
        if (mode != IDLE) { cmdReplyf("<ERR ", "first IDLE (now %d)", (int)mode); return; }
        if (startRecording()) { mode = RECORDING; updateLedForMode(); cmdReply("<OK ", "mode=RECORDING"); }
        else                  { cmdReply("<ERR ", "startRecording failed"); }
    } else if (target == "HOMING") {
        if (mode != IDLE) { cmdReplyf("<ERR ", "first IDLE (now %d)", (int)mode); return; }
        if (!homeIsSet)   { cmdReply("<ERR ", "no home saved (use >HOME)"); return; }
        if (startHoming()) { mode = HOMING; updateLedForMode(); cmdReply("<OK ", "mode=HOMING"); }
        else               { cmdReply("<ERR ", "startHoming failed"); }
    } else if (target == "PLAYBACK") {
        // Zelfde flow als BTN_PLAY kort: eerst HOMING, daarna naar PLAYBACK
        // (handlePlayback wordt bereikt zodra homing klaar is).
        if (mode != IDLE) { cmdReplyf("<ERR ", "first IDLE (now %d)", (int)mode); return; }
        if (!homeIsSet)              { cmdReply("<ERR ", "no home saved"); return; }
        if (!SD_MMC.exists(MOTION_FILE)) { cmdReply("<ERR ", "no motion file"); return; }
        if (startHoming()) { mode = HOMING; updateLedForMode(); cmdReply("<OK ", "mode=HOMING (->PLAYBACK)"); }
        else               { cmdReply("<ERR ", "startHoming failed"); }
    } else {
        cmdReply("<ERR ", "unknown mode");
    }
}

// v19+ diagnose: open-loop spin van een driver met defensive config-re-assert.
// Vereist mode=IDLE. Geen PID, geen encoder feedback — pure motor drive.
static void cmdSpin(int idx, int32_t vactual, int durMs) {
    if (mode != IDLE) { cmdReplyf("<ERR ", "SPIN only in IDLE (now %d)", (int)mode); return; }
    if (idx < 0 || idx > 2) { cmdReply("<ERR ", "idx 0..2"); return; }
    if (durMs < 0 || durMs > 30000) { cmdReply("<ERR ", "duration 0..30000ms"); return; }
    drivers[idx].setMicrostepsPerStep(256);
    drivers[idx].disableStealthChop();
    drivers[idx].enable();
    drivers[idx].moveAtVelocity(vactual);
    cmdReplyf("<OK ", "SPIN M%d v=%ld dur=%dms", idx+1, (long)vactual, durMs);
    delay(durMs);
    drivers[idx].moveAtVelocity(0);
    delay(100);
    // Driver blijft ENABLED → hold-current actief voor metingen direct na SPIN.
    cmdReplyf("<OK ", "SPIN M%d done", idx+1);
}

// v19+ diagnose: print arm-encoder raw graden + unwrapped (PID-frame) graden.
static void cmdPos(int idx) {
    if (idx < 0 || idx > 2) { cmdReply("<ERR ", "idx 0..2"); return; }
    uint16_t raw = 0;
    if (!readRawArmAngle(idx, raw)) { cmdReply("<ERR ", "encoder read fail"); return; }
    float rawDeg = rawToDeg(raw);
    float unwrappedDeg = mc[idx].armUnwrappedDeg;
    cmdReplyf("<OK ", "POS M%d raw=%.2f° unwrapped=%.2f°", idx+1, rawDeg, unwrappedDeg);
}

static void cmdTune(const String& gainKey, int idx, float val) {
    if (idx < 0 || idx > 2) { cmdReply("<ERR ", "idx must be 0..2"); return; }
    if      (gainKey == "KP") KP_SPEED[idx] = val;
    else if (gainKey == "KI") KI_SPEED[idx] = val;
    else if (gainKey == "KD") KD_SPEED[idx] = val;
    else { cmdReply("<ERR ", "gain must be KP|KI|KD"); return; }
    cmdReplyf("<OK ", "tune %s[%d]=%.4f", gainKey.c_str(), idx, val);
}

static void cmdCurrent(int idx, int pct) {
    if (idx < 0 || idx > 2) { cmdReply("<ERR ", "idx must be 0..2"); return; }
    if (pct < 0 || pct > 100) { cmdReply("<ERR ", "pct must be 0..100"); return; }
    drivers[idx].setRunCurrent((uint8_t)pct);
    currentRunPct[idx] = pct;
    // BUG FIX (2026-06-09): library's setRunCurrent veroorzaakt state-drift —
    // stealth-mode of microsteps kan resetten. Defensive re-write.
    drivers[idx].setMicrostepsPerStep(256);
    drivers[idx].disableStealthChop();
    cmdReplyf("<OK ", "current M%d run=%d%% (RAM only — CURRENTSAVE voor NVS)", idx + 1, pct);
}

static void cmdCurrentSave() {
    saveCurrentsToNVS();
    cmdReplyf("<OK ", "currents saved to NVS: [%d, %d, %d]",
              currentRunPct[0], currentRunPct[1], currentRunPct[2]);
}

static void cmdLimitSet(int idx, const String& which) {
    if (idx < 0 || idx > 2) { cmdReply("<ERR ", "idx must be 0..2"); return; }
    float pos = mc[idx].armUnwrappedDeg;
    if (which == "MIN")      lim_min_deg[idx] = pos;
    else if (which == "MAX") lim_max_deg[idx] = pos;
    else { cmdReply("<ERR ", "use MIN or MAX"); return; }
    cmdReplyf("<OK ", "lim M%d %s = %.2f° (RAM only — gebruik LIMITSAVE)",
              idx + 1, which.c_str(), pos);
}

static void cmdLimits() {
    Serial.println("<OK limits");
    for (int i = 0; i < 3; i++) {
        float cur = mc[i].armUnwrappedDeg;
        char minBuf[24], maxBuf[24];
        if (isnan(lim_min_deg[i])) snprintf(minBuf, sizeof(minBuf), "(unset)");
        else                       snprintf(minBuf, sizeof(minBuf), "%.2f°", lim_min_deg[i]);
        if (isnan(lim_max_deg[i])) snprintf(maxBuf, sizeof(maxBuf), "(unset)");
        else                       snprintf(maxBuf, sizeof(maxBuf), "%.2f°", lim_max_deg[i]);
        char line[128];
        snprintf(line, sizeof(line),
                 "  M%d cur=%.2f°   min=%s   max=%s", i + 1, cur, minBuf, maxBuf);
        Serial.println(line);
    }
}

static void cmdLimitSave() {
    saveLimitsToNVS();
    cmdReply("<OK ", "limits saved to NVS");
}

static void cmdLimitClr(int idx) {
    if (idx < 0 || idx > 2) { cmdReply("<ERR ", "idx must be 0..2"); return; }
    lim_min_deg[idx] = NAN;
    lim_max_deg[idx] = NAN;
    saveLimitsToNVS();
    cmdReplyf("<OK ", "limits M%d cleared", idx + 1);
}

// v19 debug: toon TMC2209 DRV_STATUS — thermische flags, current scaling, stallguard
// Doel: vaststellen of korte motor-uitvallen tijdens playback overcurrent/overtemperature
// protection zijn (Erik hypothese 2026-06-09).
// v19 debug: edge-triggered TMC2209 fault/temp logging.
// Polls elke ~100ms, print bij rising edge van thermal/short flags.
struct TmcFlagState { bool ot, otpw, t157, t150, t143, t120, s2g_a, s2g_b, ls_a, ls_b, ol_a, ol_b, drv_err, uv; };
static TmcFlagState lastTmcFlags[3] = {0};

static void pollTmcStatus() {
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < 100) return;
    lastMs = now;

    for (int i = 0; i < 3; i++) {
        TMC2209::Status       s  = drivers[i].getStatus();
        TMC2209::GlobalStatus gs = drivers[i].getGlobalStatus();
        TmcFlagState& prev = lastTmcFlags[i];

        #define EDGE_REPORT(flag_now, prev_field, label) \
            do { if ((flag_now) && !(prev.prev_field)) { \
                Serial.printf("!! M%d TMC %s t=%lu cs=%u\n", i+1, (label), (unsigned long)now, (unsigned)s.current_scaling); \
            } prev.prev_field = (flag_now); } while (0)

        EDGE_REPORT(s.over_temperature_shutdown, ot,    "OT SHUTDOWN (>157°C)");
        EDGE_REPORT(s.over_temperature_warning,  otpw,  "over-temp WARN (>120°C)");
        EDGE_REPORT(s.over_temperature_157c,     t157,  "temp >=157°C");
        EDGE_REPORT(s.over_temperature_150c,     t150,  "temp >=150°C");
        EDGE_REPORT(s.over_temperature_143c,     t143,  "temp >=143°C");
        EDGE_REPORT(s.over_temperature_120c,     t120,  "temp >=120°C");
        EDGE_REPORT(s.short_to_ground_a,         s2g_a, "short-to-GND coil A");
        EDGE_REPORT(s.short_to_ground_b,         s2g_b, "short-to-GND coil B");
        EDGE_REPORT(s.low_side_short_a,          ls_a,  "low-side short coil A");
        EDGE_REPORT(s.low_side_short_b,          ls_b,  "low-side short coil B");
        EDGE_REPORT(s.open_load_a,               ol_a,  "open-load coil A");
        EDGE_REPORT(s.open_load_b,               ol_b,  "open-load coil B");
        EDGE_REPORT(gs.drv_err,                  drv_err, "GLOBAL drv_err");
        EDGE_REPORT(gs.uv_cp,                    uv,    "undervoltage charge-pump");

        #undef EDGE_REPORT
    }
}

static void cmdTmcStatus() {
    Serial.println("<OK tmcstatus");
    for (int i = 0; i < 3; i++) {
        TMC2209::Status s        = drivers[i].getStatus();
        TMC2209::GlobalStatus gs = drivers[i].getGlobalStatus();
        char line[220];
        snprintf(line, sizeof(line),
                 "  M%d ot=%u otpw=%u t120=%u t143=%u t150=%u t157=%u  "
                 "cs=%u stst=%u stealth=%u  "
                 "ol=%u/%u s2g=%u/%u s2vs=%u/%u  drv_err=%u uv=%u",
                 i+1,
                 (unsigned)s.over_temperature_shutdown,
                 (unsigned)s.over_temperature_warning,
                 (unsigned)s.over_temperature_120c,
                 (unsigned)s.over_temperature_143c,
                 (unsigned)s.over_temperature_150c,
                 (unsigned)s.over_temperature_157c,
                 (unsigned)s.current_scaling,
                 (unsigned)s.standstill,
                 (unsigned)s.stealth_chop_mode,
                 (unsigned)s.open_load_a,         (unsigned)s.open_load_b,
                 (unsigned)s.short_to_ground_a,   (unsigned)s.short_to_ground_b,
                 (unsigned)s.low_side_short_a,    (unsigned)s.low_side_short_b,
                 (unsigned)gs.drv_err,
                 (unsigned)gs.uv_cp);
        Serial.println(line);
    }
}

// v19 debug: toon raw AS5600 hoeken van beide mux-kanalen per joint.
// Gebruik om te valideren of TCA_CHANNELS (zogenaamd motor) en
// TCA_ARM_CHANNELS (zogenaamd arm) fysiek de juiste sensoren aanspreken.
static void cmdEncRaw() {
    Serial.println("<OK encraw");
    for (int i = 0; i < 3; i++) {
        uint16_t rawMotor = 0, rawArm = 0;
        bool okM = readRawAngle(i, rawMotor);
        bool okA = readRawArmAngle(i, rawArm);
        char line[160];
        if (okM && okA) {
            snprintf(line, sizeof(line),
                     "  M%d motor_ch%u raw=%4u (%.2f°)   arm_ch%u raw=%4u (%.2f°)",
                     i + 1,
                     (unsigned)TCA_CHANNELS[i],     (unsigned)rawMotor, rawToDeg(rawMotor),
                     (unsigned)TCA_ARM_CHANNELS[i], (unsigned)rawArm,   rawToDeg(rawArm));
        } else {
            snprintf(line, sizeof(line),
                     "  M%d motor_ch%u %s   arm_ch%u %s",
                     i + 1,
                     (unsigned)TCA_CHANNELS[i],     okM ? "OK" : "FAIL",
                     (unsigned)TCA_ARM_CHANNELS[i], okA ? "OK" : "FAIL");
        }
        Serial.println(line);
    }
}

static void cmdHome() {
    if (mode != IDLE) { cmdReplyf("<ERR ", "home only in IDLE (now %d)", (int)mode); return; }
    uint16_t rawMotor[3] = {0,0,0};
    uint16_t rawArm[3]   = {0,0,0};
    for (int i = 0; i < 3; i++) {
        if (!readRawAngle(i, rawMotor[i]) || !readRawArmAngle(i, rawArm[i])) {
            cmdReplyf("<ERR ", "sensor read fail (joint %d)", i);
            return;
        }
    }
    saveHomeToNVS(rawMotor, rawArm);
    for (int i = 0; i < 3; i++) {
        mc[i].unwrappedDeg     = 0.0f;
        mc[i].lastDeg          = rawToDeg(rawMotor[i]);
        encValidFromHome[i]    = true;
        mc[i].armUnwrappedDeg  = 0.0f;
        mc[i].armLastDeg       = rawToDeg(rawArm[i]);
        mc[i].armEncZeroOffset = 0;
    }
    ledSet(LED_FLASH3, CRGB(80, 140, 255));
    cmdReply("<OK ", "home saved");
}

static void cmdDel() {
    if (mode == RECORDING) { stopRecording(); mode = IDLE; updateLedForMode(); }
    if (mode == PLAYBACK)  { stopPlayback();  mode = IDLE; updateLedForMode(); }
    deleteMotionFile();
    cmdReply("<OK ", "motion file deleted");
}

static void processCmdLine(const String& line) {
    if (line.length() == 0 || line[0] != '>') return;
    int sp1 = line.indexOf(' ');
    String verb = (sp1 < 0) ? line.substring(1) : line.substring(1, sp1);
    String rest = (sp1 < 0) ? String("")        : line.substring(sp1 + 1);
    verb.toUpperCase();

    if      (verb == "HELP")   cmdHelp();
    else if (verb == "STATUS") cmdStatus();
    else if (verb == "HOME")   cmdHome();
    else if (verb == "DEL")    cmdDel();
    else if (verb == "MODE")   { String m = rest; m.trim(); m.toUpperCase(); cmdMode(m); }
    else if (verb == "TUNE") {
        rest.trim();
        int s1 = rest.indexOf(' ');
        int s2 = (s1 < 0) ? -1 : rest.indexOf(' ', s1 + 1);
        if (s1 < 0 || s2 < 0) { cmdReply("<ERR ", "usage: TUNE KP|KI|KD <i> <v>"); return; }
        String gain = rest.substring(0, s1); gain.toUpperCase();
        int    idx  = rest.substring(s1 + 1, s2).toInt();
        float  val  = rest.substring(s2 + 1).toFloat();
        cmdTune(gain, idx, val);
    }
    else if (verb == "CURRENT") {
        rest.trim();
        int s1 = rest.indexOf(' ');
        if (s1 < 0) { cmdReply("<ERR ", "usage: CURRENT <i> <pct>"); return; }
        int idx = rest.substring(0, s1).toInt();
        int pct = rest.substring(s1 + 1).toInt();
        cmdCurrent(idx, pct);
    }
    else if (verb == "LIMITS")    cmdLimits();
    else if (verb == "LIMITSAVE") cmdLimitSave();
    else if (verb == "CURRENTSAVE") cmdCurrentSave();
    else if (verb == "ENCRAW")    cmdEncRaw();
    else if (verb == "TMCSTATUS") cmdTmcStatus();
    else if (verb == "POS") {
        rest.trim();
        if (rest.length() == 0) { cmdReply("<ERR ", "usage: POS <i>"); return; }
        cmdPos(rest.toInt());
    }
    else if (verb == "SPIN") {
        rest.trim();
        int s1 = rest.indexOf(' ');
        int s2 = (s1 < 0) ? -1 : rest.indexOf(' ', s1 + 1);
        if (s1 < 0 || s2 < 0) { cmdReply("<ERR ", "usage: SPIN <i> <vactual> <duration_ms>"); return; }
        int idx = rest.substring(0, s1).toInt();
        long vac = rest.substring(s1 + 1, s2).toInt();
        int dur = rest.substring(s2 + 1).toInt();
        cmdSpin(idx, (int32_t)vac, dur);
    }
    else if (verb == "LIMITCLR") {
        rest.trim();
        if (rest.length() == 0) { cmdReply("<ERR ", "usage: LIMITCLR <i>"); return; }
        cmdLimitClr(rest.toInt());
    }
    else if (verb == "LIMITSET") {
        rest.trim();
        int s1 = rest.indexOf(' ');
        if (s1 < 0) { cmdReply("<ERR ", "usage: LIMITSET <i> MIN|MAX"); return; }
        int idx = rest.substring(0, s1).toInt();
        String w = rest.substring(s1 + 1); w.trim(); w.toUpperCase();
        cmdLimitSet(idx, w);
    }
    else {
        cmdReplyf("<ERR ", "unknown verb '%s' (>HELP)", verb.c_str());
    }
}

static void serialCmdPoll() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            String line = cmdLineBuffer;
            cmdLineBuffer = "";
            line.trim();
            if (line.length() > 0) processCmdLine(line);
        } else {
            if (cmdLineBuffer.length() < 200) cmdLineBuffer += c;
            else cmdLineBuffer = "";  // overflow guard
        }
    }
}

// ─────────────────────────── SETUP / LOOP ───────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // LED
    FastLED.addLeds<LED_CHIPSET, LED_PIN, LED_COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Black; FastLED.show();

    // Knoppen
    pinMode(BTN_REC,  INPUT_PULLUP);
    pinMode(BTN_PLAY, INPUT_PULLUP);
    pinMode(BTN_DEL,  INPUT_PULLUP);

    // Motor EN pins worden beheerd via drivers[i].enable()/disable()

    // I2C
    Wire.begin();   // Qwiic: SDA=GPIO8, SCL=GPIO9 (board defaults)
    Wire.setClock(400000);

    // TCA9548A
    Wire.beginTransmission(TCA_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("TCA9548A not found!");
        ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0));
        while (true) { ledUpdate(); delay(10); }
    }
    Serial.printf("TCA9548A found at 0x%02X\n", TCA_ADDR);

    // Motor-as AS5600 encoders
    Serial.println("Motor-as encoders:");
    for (int i = 0; i < 3; i++) {
        uint16_t raw = 0;
        if (!readRawAngle(i, raw)) {
            Serial.printf("  M%d motor-sensor niet gevonden!\n", i+1);
            ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0));
            while (true) { ledUpdate(); delay(10); }
        }
        Serial.printf("  M%d OK: %.1f°\n", i+1, rawToDeg(raw));
        mc[i].lastDeg = rawToDeg(raw);   // delta-tracker basis
    }

    // v11: Arm-as AS5600 encoders
    Serial.println("Arm-as encoders:");
    for (int i = 0; i < 3; i++) {
        uint16_t raw = 0;
        if (!readRawArmAngle(i, raw)) {
            Serial.printf("  M%d arm-sensor niet gevonden!\n", i+1);
            ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0));
            while (true) { ledUpdate(); delay(10); }
        }
        Serial.printf("  M%d OK: %.1f°\n", i+1, rawToDeg(raw));
        mc[i].armLastDeg = rawToDeg(raw);   // v13: delta-tracker basis arm-encoder
    }

    // SD
    if (!initSD()) {
        Serial.println("SD init failed.");
        ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0));
        while (true) { ledUpdate(); delay(10); }
    }

    // NVS home laden
    loadHomeFromNVS();
    loadLimitsFromNVS();   // v19: soft endstops
    loadCurrentsFromNVS(); // v19+ 2026-06-09: per-joint run-current

    // v11: Vernier absolute positiebepaling
    // Werkt altijd correct ongeacht waar de arm staat na een reset,
    // zolang alle assen binnen 720° arm bewegen (geconfirmeerd).
    // Geen NVS positieopslag of ±51.4° limiet meer nodig.
    if (homeIsSet) {
        if (!initFromVernier()) {
            // Vernier mislukt (sensor defect?) — fallback naar raw-angle
            Serial.println("Vernier mislukt — fallback raw-angle (±51.4° arm limiet).");
            for (int i = 0; i < 3; i++) {
                float diff = angleDiff(mc[i].lastDeg, rawToDeg(homeAngleRaw[i]));
                mc[i].unwrappedDeg  = diff;
                encValidFromHome[i] = true;
                // v13: arm-encoder state ook initialiseren bij fallback
                uint16_t armRaw = 0;
                if (readRawArmAngle(i, armRaw)) {
                    mc[i].armLastDeg       = rawToDeg(armRaw);
                    mc[i].armUnwrappedDeg  = ARM_ENC_DIR * (diff / GEAR_RATIO);  // sensor-coords
                    mc[i].armEncZeroOffset = 0;
                }
            }
        }
    }

    // v14: TMC2209 drivers initialiseren via UART
    // Pins doorgeven aan setup() — library initialiseert Serial1 intern
    for (int i = 0; i < 3; i++) {
        drivers[i].setup(TMCSerial, 115200, TMC_ADDR[i], UART_RX_PIN, UART_TX_PIN);
        drivers[i].setHardwareEnablePin(UART_EN_PIN);
        drivers[i].setRunCurrent((uint8_t)currentRunPct[i]);
        drivers[i].setHoldCurrent(TMC_HOLD_CURRENT);
        drivers[i].setMicrostepsPerStep(256);
        drivers[i].enableAutomaticCurrentScaling();
        drivers[i].enableAutomaticGradientAdaptation();
        drivers[i].disableStealthChop();  // task #3: spreadCycle alleen — debug van uitvallende motoren
        drivers[i].setStandstillMode(TMC2209::FREEWHEELING);
        drivers[i].disable();   // uit bij opstart — enable() bij homing
        delay(50);
        if (!drivers[i].isSetupAndCommunicating()) {
            Serial.printf("TMC2209 M%d (addr%d): GEEN COMMUNICATIE!\n", i+1, i);
            ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0));
            while (true) { ledUpdate(); delay(10); }
        }
        Serial.printf("TMC2209 M%d OK\n", i+1);
    }

    // Opstartanimatie
    leds[0] = CRGB(80, 80, 80); FastLED.show(); delay(300);
    leds[0] = CRGB::Black;      FastLED.show(); delay(100);
    updateLedForMode();

    Serial.println("─────────────────────────────────────────────");
    Serial.println("  Robotarm v18  –  3-as Teach & Repeat  (3.5:1 tandriem)");
    Serial.println("  Aansturing: TMC2209 UART/VACTUAL (256 microsteps)");
    Serial.println("  M1=200-staps(1.8°/stap)  M2=200-staps(1.8°/stap)  M3=400-staps(0.9°/stap)");
    Serial.println("  PID-bron:   arm-as encoder (3.5x hogere resolutie)");
    Serial.println("  Vernier:    6x AS5600 (motor + arm assen)");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("  BTN_REC  (GPIO4) kort : record aan/uit");
    Serial.println("  BTN_PLAY (GPIO2) kort : homing + playback");
    Serial.println("  BTN_PLAY (GPIO2) lang : sla home op (alle 3 assen)");
    Serial.println("  BTN_DEL  (GPIO1) lang : verwijder motion file");
    Serial.println("  Let op: v14.2 gebruikt 400-step schaal → oude motion file opnieuw opnemen");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("  v19: serial commando's actief — typ '>HELP' voor overzicht");
    Serial.println("─────────────────────────────────────────────");
    if (!homeIsSet) Serial.println("  !! Stel eerst home in via lang BTN_PLAY");
}

void loop() {
    static uint32_t delFlashUntil = 0;

    // v19: host commando's eerst — niet-blocking, geen effect bij geen input
    serialCmdPoll();

    // v19: TMC2209 fault/temp edge-detect (logs naar serial bij rising edge)
    pollTmcStatus();

    // ── BTN_DEL lang ────────────────────────────────────────
    if (lpDel.update(BTN_DEL, DEL_HOLD_MS)) {
        if (mode == RECORDING) { stopRecording(); mode = IDLE; }
        if (mode == PLAYBACK)  { stopPlayback();  mode = IDLE; }
        if (mode == HOMING) {
            for (int i = 0; i < 3; i++) {
                setMotorSpeed(i, 0.0f);
                drivers[i].disable();
            }
            mode = IDLE;
        }
        deleteMotionFile();
        ledSet(LED_BLINK_1HZ, CRGB(255, 0, 0));
        delFlashUntil = millis() + 1500;
    }
    if (delFlashUntil > 0 && millis() >= delFlashUntil) {
        delFlashUntil = 0; updateLedForMode();
    }

    // ── BTN_PLAY lang: home opslaan ──────────────────────────
    if (mode == IDLE) {
        if (lpPlay.isHolding())
            ledSet(LED_PROGRESS, CRGB(255, 220, 0), lpPlay.progress(HOME_HOLD_MS));

        if (lpPlay.update(BTN_PLAY, HOME_HOLD_MS)) {
            // v11: lees zowel motor- als arm-sensor bij home opslaan
            uint16_t rawMotor[3] = {0, 0, 0};
            uint16_t rawArm[3]   = {0, 0, 0};
            bool ok = true;
            for (int i = 0; i < 3; i++) {
                if (!readRawAngle(i,    rawMotor[i])) { ok = false; break; }
                if (!readRawArmAngle(i, rawArm[i]))   { ok = false; break; }
            }
            if (ok) {
                saveHomeToNVS(rawMotor, rawArm);
                // Encoder direct bijwerken — positie = 0 = home
                for (int i = 0; i < 3; i++) {
                    mc[i].unwrappedDeg  = 0.0f;
                    mc[i].lastDeg       = rawToDeg(rawMotor[i]);
                    encValidFromHome[i] = true;
                    // v13: arm-encoder ook direct nulstellen op home-positie
                    mc[i].armUnwrappedDeg  = 0.0f;
                    mc[i].armLastDeg       = rawToDeg(rawArm[i]);
                    mc[i].armEncZeroOffset = 0;
                }
                ledSet(LED_FLASH3, CRGB(80, 140, 255));
            } else {
                Serial.println("ERROR: sensor niet leesbaar bij home opslaan.");
                ledSet(LED_BLINK_5HZ, CRGB(255, 0, 0));
            }
        } else if (!lpPlay.isHolding()) {
            if (delFlashUntil == 0) updateLedForMode();
        }
    } else {
        lpPlay.update(BTN_PLAY, HOME_HOLD_MS);
    }

    // ── BTN_PLAY kort: homing + playback ────────────────────
    if (fellEdge(BTN_PLAY)) {
        if (mode == IDLE) {
            if (!homeIsSet) {
                Serial.println("!! Geen home. Houd BTN_PLAY lang in.");
                ledSet(LED_BLINK_5HZ, CRGB(255, 80, 0)); delay(1000); updateLedForMode();
            } else if (!SD_MMC.exists(MOTION_FILE)) {
                Serial.println("!! Geen motion file. Neem eerst op.");
                ledSet(LED_BLINK_5HZ, CRGB(255, 80, 0)); delay(1000); updateLedForMode();
            } else {
                if (startHoming()) { mode = HOMING; updateLedForMode(); }
            }
        }
    }

    // ── BTN_REC kort ────────────────────────────────────────
    if (fellEdge(BTN_REC)) {
        if      (mode == RECORDING) { stopRecording(); mode = IDLE; updateLedForMode(); }
        else if (mode == IDLE)      { if (startRecording()) { mode = RECORDING; updateLedForMode(); } }
    }

    // ── Mode handlers ────────────────────────────────────────
    if (mode == RECORDING) handleRecording();
    if (mode == HOMING)    handleHoming();
    if (mode == PLAYBACK)  handlePlayback();

    // ── IDLE: encoder continu bijlezen voor multi-turn tracking ──
    if (mode == IDLE) {
        static uint32_t lastIdleEncMs = 0;
        uint32_t nowIdle = millis();
        if (nowIdle - lastIdleEncMs >= SAMPLE_MS) {
            lastIdleEncMs = nowIdle;
            for (int i = 0; i < 3; i++) {
                long s;
                readEncoderSteps(i, s);      // motor-enc: Vernier multi-turn tracking
                readArmEncoderSteps(i, s);   // v13: arm-enc: PID-continuïteit
            }
        }
    }

    // ── LED ───────────────────────────────────────────────────
    ledUpdate();
}
