#include "motors.h"

#include <Arduino.h>

#include "config.h"

namespace motors {

namespace {

struct DriveBridge {
  uint8_t r_en;
  uint8_t l_en;
  uint8_t rpwm;
  uint8_t lpwm;
};

constexpr uint8_t BRIDGE_COUNT = 4;

const DriveBridge bridges[BRIDGE_COUNT] = {
    {config::PIN_FL_R_EN, config::PIN_FL_L_EN, config::PIN_FL_RPWM, config::PIN_FL_LPWM},
    {config::PIN_FR_R_EN, config::PIN_FR_L_EN, config::PIN_FR_RPWM, config::PIN_FR_LPWM},
    {config::PIN_RL_R_EN, config::PIN_RL_L_EN, config::PIN_RL_RPWM, config::PIN_RL_LPWM},
    {config::PIN_RR_R_EN, config::PIN_RR_L_EN, config::PIN_RR_RPWM, config::PIN_RR_LPWM},
};

uint8_t target_pwm = 0;
uint8_t current_pwm = 0;
bool enabled = false;

void write_all_pwm(uint8_t value) {
  for (uint8_t i = 0; i < BRIDGE_COUNT; ++i) {
    analogWrite(bridges[i].rpwm, value);
    analogWrite(bridges[i].lpwm, 0);
  }
}

void write_all_enable(bool on) {
  for (uint8_t i = 0; i < BRIDGE_COUNT; ++i) {
    digitalWrite(bridges[i].r_en, on ? HIGH : LOW);
    digitalWrite(bridges[i].l_en, on ? HIGH : LOW);
  }
}

}  // namespace

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

void set_throttle(uint8_t throttle) {
  target_pwm = throttle;
}

uint8_t current_throttle() {
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
  // Asymmetric slew: ramp up slowly (gentle on the gearbox + avoids brownouts),
  // ramp down fast so estop and watchdog stops feel crisp.
  int16_t delta = (int16_t)target_pwm - (int16_t)current_pwm;
  if (delta > 0) {
    if (delta > config::DRIVE_SLEW_UP_PER_TICK) delta = config::DRIVE_SLEW_UP_PER_TICK;
  } else if (delta < 0) {
    if (delta < -(int16_t)config::DRIVE_SLEW_DOWN_PER_TICK) {
      delta = -(int16_t)config::DRIVE_SLEW_DOWN_PER_TICK;
    }
  }
  current_pwm = (uint8_t)((int16_t)current_pwm + delta);

  if (!enabled) {
    write_all_pwm(0);
    return;
  }
  write_all_pwm(current_pwm);
}

void emergency_stop() {
  disable();
}

}  // namespace motors
