#pragma once
#include <Arduino.h>

namespace cfg {

// Joints
constexpr int NUM_JOINTS = 3;

// Stepper / belt
constexpr long  STEPS_PER_REV  = 400;   // M1 was 200 in v18, v19 normaliseert via VACTUAL_CORR
constexpr float GEAR_RATIO     = 3.5f;
constexpr int   MICROSTEPS     = 256;

// TMC2209 UART
constexpr int   TMC_TX_PIN     = 17;
constexpr int   TMC_RX_PIN     = 16;
constexpr int   TMC_EN_PIN     = 15;
constexpr float TMC_R_SENSE    = 0.11f;
constexpr int   TMC_DEFAULT_CURRENT_PCT = 60;

// TMC adressen (MS1/MS2 jumpers): M1=00, M2=10, M3=01
constexpr uint8_t TMC_ADDRS[NUM_JOINTS] = { 0b00, 0b10, 0b01 };

// I2C / TCA9548A multiplexer
constexpr uint8_t TCA_ADDR     = 0x70;

// TCA-mapping bevestigd via fysieke bekabeling 2026-06-09.
// M2 motor↔arm waren in v19 omgewisseld; hier correct.
constexpr uint8_t TCA_MOTOR_CHANNELS[NUM_JOINTS] = { 0, 2, 7 };  // M1, M2, M3 motor-as encoder
constexpr uint8_t TCA_ARM_CHANNELS  [NUM_JOINTS] = { 1, 3, 6 };  // M1, M2, M3 arm-as encoder

// Encoder direction corrections
constexpr int   ENC_DIR        = -1;
constexpr int   ARM_ENC_DIR    = -1;

// Motor VACTUAL correctie (sign/scale per joint, na microstep)
constexpr float MOTOR_VACTUAL_CORR[NUM_JOINTS] = { -0.5f, 0.5f, 0.5f };

// Sample timing
constexpr uint32_t SAMPLE_MS              = 10;    // 100 Hz hoofd-loop
constexpr uint32_t TMC_STATUS_POLL_MS     = 100;   // 10 Hz TMC edge-detect

// Recording filter (low-pass alpha; 1.0 = geen filter)
constexpr float    REC_FILTER_ALPHA       = 0.85f;
constexpr long     REC_HOME_TOLERANCE_STEPS = 100;

// PID filter (low-pass op encoder voor PID-bron)
constexpr float    PID_FILTER_ALPHA       = 0.25f;

// Knoppen (GPIO)
constexpr int BTN_REC_PIN   = 4;
constexpr int BTN_PLAY_PIN  = 2;
constexpr int BTN_DEL_PIN   = 1;
constexpr uint32_t BTN_HOLD_MS  = 1500;  // lang-druk drempel
constexpr uint32_t DEL_HOLD_MS  = 2000;

// LED (FastLED) — matched to v19
constexpr int  LED_PIN        = 46;
constexpr int  LED_COUNT      = 1;
constexpr int  LED_BRIGHTNESS = 80;

// SD_MMC pin-config voor SparkFun ESP32-S3 Thing Plus (niet ESP32-S3 defaults)
constexpr int SD_CLK = 38;
constexpr int SD_CMD = 34;
constexpr int SD_D0  = 39;
constexpr int SD_D1  = 40;
constexpr int SD_D2  = 47;
constexpr int SD_D3  = 33;
constexpr int SD_DET = 48;  // card-detect (INPUT_PULLDOWN)

// Files
constexpr const char* MOTION_FILE = "/motion.csv";
constexpr const char* NVS_NS      = "armv19";

// ─── Afgeleide constants ─────────────────────────────────────
constexpr long STEPS_PER_ARM_REV = STEPS_PER_REV * (long)GEAR_RATIO;  // 1400 stappen = 1 arm-rev

} // namespace cfg
