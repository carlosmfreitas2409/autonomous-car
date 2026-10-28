#include "motors.h"

#include <Arduino.h>

#include "config.h"
#include "steering.h"

namespace motors {

namespace {

struct DriveBridge {
  uint8_t r_en;
  uint8_t l_en;
  uint8_t rpwm;
  uint8_t lpwm;
  bool left_side;
};

constexpr uint8_t BRIDGE_COUNT = 4;

const DriveBridge bridges[BRIDGE_COUNT] = {
    {config::PIN_FL_R_EN, config::PIN_FL_L_EN, config::PIN_FL_RPWM, config::PIN_FL_LPWM, true},
    {config::PIN_FR_R_EN, config::PIN_FR_L_EN, config::PIN_FR_RPWM, config::PIN_FR_LPWM, false},
    {config::PIN_RL_R_EN, config::PIN_RL_L_EN, config::PIN_RL_RPWM, config::PIN_RL_LPWM, true},
    {config::PIN_RR_R_EN, config::PIN_RR_L_EN, config::PIN_RR_RPWM, config::PIN_RR_LPWM, false},
};

int16_t target_pwm = 0;
int16_t current_pwm = 0;
bool enabled = false;

void write_bridge(uint8_t i, int16_t value) {
  const uint8_t magnitude = (uint8_t)(value < 0 ? -value : value);
  const bool reverse = value < 0;
  analogWrite(bridges[i].rpwm, reverse ? 0 : magnitude);
  analogWrite(bridges[i].lpwm, reverse ? magnitude : 0);
}

void write_all_pwm(int16_t value) {
  for (uint8_t i = 0; i < BRIDGE_COUNT; ++i) {
    write_bridge(i, value);
  }
}

void write_drive_with_differential(int16_t base) {
  const float steer = steering::normalized_position();
  const float mag = steer < 0.0f ? -steer : steer;
  float inner_factor = 1.0f - config::DRIVE_DIFF_GAIN * mag;
  if (inner_factor < config::DRIVE_DIFF_MIN_FACTOR) inner_factor = config::DRIVE_DIFF_MIN_FACTOR;

  const bool left_is_inner = steer > 0.0f;
  for (uint8_t i = 0; i < BRIDGE_COUNT; ++i) {
    const float factor = (bridges[i].left_side == left_is_inner) ? inner_factor : 1.0f;
    write_bridge(i, (int16_t)(base * factor));
  }
}

void write_all_enable(bool on) {
  for (uint8_t i = 0; i < BRIDGE_COUNT; ++i) {
    digitalWrite(bridges[i].r_en, on ? HIGH : LOW);
    digitalWrite(bridges[i].l_en, on ? HIGH : LOW);
  }
}

}

void init() {
  for (uint8_t i = 0; i < BRIDGE_COUNT; ++i) {
    pinMode(bridges[i].r_en, OUTPUT);
    pinMode(bridges[i].l_en, OUTPUT);
    pinMode(bridges[i].rpwm, OUTPUT);
    pinMode(bridges[i].lpwm, OUTPUT);
  }
  write_all_enable(false);
  write_all_pwm(0);
  target_pwm = 0;
  current_pwm = 0;
  enabled = false;
}

void set_throttle(int16_t throttle) {
  if (throttle > 255) throttle = 255;
  if (throttle < -255) throttle = -255;
  target_pwm = throttle;
}

int16_t current_throttle() {
  return current_pwm;
}

void enable() {
  enabled = true;
  write_all_enable(true);
}

void disable() {
  enabled = false;
  target_pwm = 0;
  current_pwm = 0;
  write_all_pwm(0);
  write_all_enable(false);
}

void update() {
  const int16_t target_mag = target_pwm < 0 ? -target_pwm : target_pwm;
  const int16_t current_mag = current_pwm < 0 ? -current_pwm : current_pwm;
  const bool same_direction = (target_pwm >= 0) == (current_pwm >= 0);
  const bool ramping_up = same_direction && target_mag > current_mag;
  const int16_t step = ramping_up ? config::DRIVE_SLEW_UP_PER_TICK
                                  : config::DRIVE_SLEW_DOWN_PER_TICK;

  int16_t delta = target_pwm - current_pwm;
  if (delta > step) delta = step;
  else if (delta < -step) delta = -step;
  current_pwm += delta;

  if (!enabled) {
    write_all_pwm(0);
    return;
  }
  write_drive_with_differential(current_pwm);
}

void emergency_stop() {
  disable();
}

}
