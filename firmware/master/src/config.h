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

constexpr uint8_t PIN_STATUS_LED = 13;

constexpr uint16_t CONTROL_LOOP_HZ = 200;
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 1000 / CONTROL_LOOP_HZ;

constexpr uint16_t WATCHDOG_TIMEOUT_MS = 500;

constexpr uint8_t DRIVE_SLEW_UP_PER_TICK = 4;
constexpr uint8_t DRIVE_SLEW_DOWN_PER_TICK = 32;

constexpr float STEER_KP = 0.6f;
constexpr float STEER_KD = 0.02f;
constexpr int16_t STEER_DEADBAND_UNITS = 6;
constexpr uint8_t STEER_MIN_MOVE_PWM = 60;
constexpr uint8_t STEER_MAX_PWM = 200;

constexpr uint16_t STEER_CENTER_RAW = 2048;
constexpr uint16_t STEER_MAX_LEFT_RAW = 1536;
constexpr uint16_t STEER_MAX_RIGHT_RAW = 2560;

constexpr uint16_t TELEMETRY_PERIOD_MS = 100;

}  // namespace config
