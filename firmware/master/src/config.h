#pragma once

#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint8_t PIN_FL_R_EN = 24;
constexpr uint8_t PIN_FL_L_EN = 25;
constexpr uint8_t PIN_FL_RPWM = 5;
constexpr uint8_t PIN_FL_LPWM = 6;

constexpr uint8_t PIN_FR_R_EN = 26;
constexpr uint8_t PIN_FR_L_EN = 27;
constexpr uint8_t PIN_FR_RPWM = 7;
constexpr uint8_t PIN_FR_LPWM = 8;

constexpr uint8_t PIN_RL_R_EN = 22;
constexpr uint8_t PIN_RL_L_EN = 23;
constexpr uint8_t PIN_RL_RPWM = 3;
constexpr uint8_t PIN_RL_LPWM = 4;

constexpr uint8_t PIN_RR_R_EN = 28;
constexpr uint8_t PIN_RR_L_EN = 29;
constexpr uint8_t PIN_RR_RPWM = 9;
constexpr uint8_t PIN_RR_LPWM = 10;

constexpr uint8_t PIN_STEER_R_EN = 30;
constexpr uint8_t PIN_STEER_L_EN = 31;
constexpr uint8_t PIN_STEER_RPWM = 11;
constexpr uint8_t PIN_STEER_LPWM = 12;

// Wiper of the 10k linear pot (red/yellow) that reads wheel position.
constexpr uint8_t PIN_STEER_POT = A11;

constexpr uint8_t PIN_STATUS_LED = 13;

constexpr uint8_t PIN_CAN_CS = 53;

constexpr uint16_t CAN_LIGHTS_ID = 0x100;

constexpr uint16_t CONTROL_LOOP_HZ = 200;
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 1000 / CONTROL_LOOP_HZ;

constexpr uint16_t WATCHDOG_TIMEOUT_MS = 500;

constexpr uint8_t DRIVE_SLEW_UP_PER_TICK = 4;
constexpr uint8_t DRIVE_SLEW_DOWN_PER_TICK = 32;

// Steering feedback "raw" units are now the 10k pot's 10-bit ADC counts (0..1023,
// linear), not the old AS5600's 0..4095. A given physical steering angle spans
// ~4x fewer counts, so Kp is scaled up and the deadband down accordingly. Kd is
// kept modest on purpose: the pot is electrically noisy and the derivative term
// amplifies that noise into audible PWM chatter (the read is also low-pass
// filtered in steering.cpp). These are bench-tuning starting points — see
// config-params.md.
constexpr float STEER_KP = 2.2f;
constexpr float STEER_KD = 0.05f;
constexpr int16_t STEER_DEADBAND_UNITS = 3;
constexpr uint8_t STEER_MIN_MOVE_PWM = 150;
constexpr uint8_t STEER_MAX_PWM = 210;

// Pot ADC range is 0..1023. RUN tools/calibration_pot/calibration_pot.ino and
// replace these three values before trusting the steering — they are
// conservative placeholders. Left/right may need swapping depending on how the
// pot couples to the linkage.
constexpr uint16_t STEER_CENTER_RAW    = 434;
constexpr uint16_t STEER_MAX_LEFT_RAW  = 1000;
constexpr uint16_t STEER_MAX_RIGHT_RAW = 10;  

// The control loop assumes positive output should raise the raw pot reading. If
// positive PWM on PIN_STEER_RPWM instead lowers the reading (depends on motor
// lead polarity and how the pot is mounted), invert the H-bridge direction here.
// Confirm the sign during the bench step-response test and flip if the wheels
// move the wrong way.
constexpr bool STEER_INVERT_PWM = true;

constexpr uint16_t TELEMETRY_PERIOD_MS = 100;

}  // namespace config
