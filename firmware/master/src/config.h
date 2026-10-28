#pragma once

#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint8_t PIN_FL_R_EN = 24;
constexpr uint8_t PIN_FL_L_EN = 25;
constexpr uint8_t PIN_FL_RPWM = 6;
constexpr uint8_t PIN_FL_LPWM = 5;

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

constexpr uint8_t PIN_BUZZER = 46;
constexpr uint16_t BUZZER_TONE_HZ = 2700;
constexpr uint16_t BUZZER_BEEP_ON_MS = 350;
constexpr uint16_t BUZZER_BEEP_OFF_MS = 350;

constexpr uint8_t PIN_BATT_SENSE = A10;

constexpr float BATT_VOLTS_PER_COUNT = 12.56f / 562.0f;

constexpr uint8_t BATT_ADC_SAMPLES = 4;

constexpr uint8_t BATT_CURVE_POINTS = 8;
constexpr float BATT_CURVE_V[BATT_CURVE_POINTS] = {
    11.80f, 11.96f, 12.10f, 12.24f, 12.36f, 12.46f, 12.58f, 12.70f};
constexpr float BATT_CURVE_PCT[BATT_CURVE_POINTS] = {
    0.0f, 20.0f, 40.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f};

constexpr uint16_t BATT_SAMPLE_PERIOD_MS = 200;
constexpr float BATT_FILTER_ALPHA = 0.02f;

constexpr uint8_t BATT_LED_COUNT = 6;
constexpr uint8_t BATT_LED_PINS[BATT_LED_COUNT] = {43, 41, 39, 37, 35, 33};
constexpr float BATT_LED_THRESHOLD_PCT[BATT_LED_COUNT] = {
    15.0f, 30.0f, 50.0f, 65.0f, 80.0f, 97.0f};
constexpr float BATT_LED_HYSTERESIS_PCT = 3.0f;

constexpr uint16_t CAN_LIGHTS_ID = 0x100;

constexpr uint16_t CAN_SONAR_ID = 0x080;
constexpr uint8_t SONAR_SENSORS = 3;
constexpr uint8_t SONAR_NO_ECHO = 0xFF;

constexpr uint16_t SONAR_TIMEOUT_MS = 300;

// The cargo node (firmware/cargo), bridged onto the bus by the sonar node: its
// status is forwarded to the host as a `C` line, the host's `K` becomes its
// command.
constexpr uint16_t CAN_CARGO_STATUS_ID = 0x180;
constexpr uint16_t CAN_CARGO_COMMAND_ID = 0x181;

constexpr uint8_t STOP_CM = 60;
constexpr uint8_t SLOW_CM = 80;

constexpr uint16_t CONTROL_LOOP_HZ = 200;
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 1000 / CONTROL_LOOP_HZ;

constexpr uint16_t WATCHDOG_TIMEOUT_MS = 500;

constexpr uint8_t DRIVE_SLEW_UP_PER_TICK = 4;
constexpr uint8_t DRIVE_SLEW_DOWN_PER_TICK = 32;

// Electronic differential. With four driven wheels locked to one speed, the
// inner wheels (shorter arc) scrub and fight the turn. Slow the inner-side
// wheels in proportion to how hard the wheels are steered, leaving the outer
// side at full speed: inner_factor = 1 - DRIVE_DIFF_GAIN*|steer|, floored at
// DRIVE_DIFF_MIN_FACTOR. This only ever reduces speed, so it cannot make
// cornering worse. Steering magnitude comes from the measured pot angle.
constexpr float DRIVE_DIFF_GAIN = 0.5f;
constexpr float DRIVE_DIFF_MIN_FACTOR = 0.4f;

// Steering feedback "raw" units are now the 10k pot's 10-bit ADC counts (0..1023,
// linear), not the old AS5600's 0..4095. A given physical steering angle spans
// ~4x fewer counts, so Kp/Kd are scaled up and the deadband down accordingly.
// These are bench-tuning starting points — see config-params.md.
constexpr float STEER_KP = 2.2f;
constexpr float STEER_KD = 0.12f;
constexpr int16_t STEER_DEADBAND_UNITS = 3;
constexpr uint8_t STEER_MIN_MOVE_PWM = 150;
constexpr uint8_t STEER_MAX_PWM = 210;

// Pot ADC range is 0..1023. RUN tools/calibration_pot/calibration_pot.ino and
// replace these three values before trusting the steering — they are
// conservative placeholders. Left/right may need swapping depending on how the
// pot couples to the linkage.
constexpr uint16_t STEER_CENTER_RAW    = 499;
constexpr uint16_t STEER_MAX_LEFT_RAW  = 840;
constexpr uint16_t STEER_MAX_RIGHT_RAW = 155;

// The control loop assumes positive output should raise the raw pot reading. If
// positive PWM on PIN_STEER_RPWM instead lowers the reading (depends on motor
// lead polarity and how the pot is mounted), invert the H-bridge direction here.
// Confirm the sign during the bench step-response test and flip if the wheels
// move the wrong way.
constexpr bool STEER_INVERT_PWM = true;

// 20 Hz, the sonar node's CAN rate: at 10 Hz every other sonar frame was
// dropped, and the host's obstacle filter needs every echo a side sensor gets.
constexpr uint16_t TELEMETRY_PERIOD_MS = 50;
// The battery changes slowly (its reading is a ~10 s average): once a second.
constexpr uint16_t BATT_TELEMETRY_PERIOD_MS = 1000;

}  // namespace config
