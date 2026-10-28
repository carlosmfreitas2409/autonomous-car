#include "steering.h"

#include <Arduino.h>

#include "config.h"

namespace steering {

namespace {
constexpr int16_t POT_RAW_MAX = 1023;

constexpr uint8_t POT_SAMPLES = 9;

int16_t left_span = 0;
int16_t right_span = 0;

uint16_t target_angle = config::STEER_CENTER_RAW;
uint16_t current_angle = config::STEER_CENTER_RAW;

bool enabled = false;
int16_t last_output_pwm = 0;

uint16_t prev_measurement = config::STEER_CENTER_RAW;
uint32_t prev_micros = 0;

uint16_t clamp_raw(int32_t v) {
  if (v < 0) return 0;
  if (v > POT_RAW_MAX) return (uint16_t)POT_RAW_MAX;
  return (uint16_t)v;
}

void drive_pwm(int16_t signed_pwm) {
  if (config::STEER_INVERT_PWM) signed_pwm = -signed_pwm;

  if (signed_pwm > config::STEER_MAX_PWM) signed_pwm = config::STEER_MAX_PWM;
  if (signed_pwm < -(int16_t)config::STEER_MAX_PWM) signed_pwm = -(int16_t)config::STEER_MAX_PWM;
  last_output_pwm = signed_pwm;

  if (!enabled) {
    analogWrite(config::PIN_STEER_RPWM, 0);
    analogWrite(config::PIN_STEER_LPWM, 0);
    return;
  }

  if (signed_pwm > 0) {
    analogWrite(config::PIN_STEER_RPWM, (uint8_t)signed_pwm);
    analogWrite(config::PIN_STEER_LPWM, 0);
  } else if (signed_pwm < 0) {
    analogWrite(config::PIN_STEER_RPWM, 0);
    analogWrite(config::PIN_STEER_LPWM, (uint8_t)(-signed_pwm));
  } else {
    analogWrite(config::PIN_STEER_RPWM, 0);
    analogWrite(config::PIN_STEER_LPWM, 0);
  }
}

uint16_t read_angle() {
  uint16_t samples[POT_SAMPLES];
  for (uint8_t i = 0; i < POT_SAMPLES; ++i) {
    samples[i] = (uint16_t)analogRead(config::PIN_STEER_POT);
  }
  for (uint8_t i = 1; i < POT_SAMPLES; ++i) {
    uint16_t key = samples[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = key;
  }
  return samples[POT_SAMPLES / 2];
}

}

bool init() {
  pinMode(config::PIN_STEER_POT, INPUT);

  pinMode(config::PIN_STEER_R_EN, OUTPUT);
  pinMode(config::PIN_STEER_L_EN, OUTPUT);
  pinMode(config::PIN_STEER_RPWM, OUTPUT);
  pinMode(config::PIN_STEER_LPWM, OUTPUT);

  digitalWrite(config::PIN_STEER_R_EN, LOW);
  digitalWrite(config::PIN_STEER_L_EN, LOW);
  analogWrite(config::PIN_STEER_RPWM, 0);
  analogWrite(config::PIN_STEER_LPWM, 0);

  uint16_t a = read_angle();
  if (a == 0 || a >= (uint16_t)POT_RAW_MAX) return false;

  left_span = (int16_t)config::STEER_MAX_LEFT_RAW - (int16_t)config::STEER_CENTER_RAW;
  right_span = (int16_t)config::STEER_MAX_RIGHT_RAW - (int16_t)config::STEER_CENTER_RAW;

  current_angle = a;
  prev_measurement = current_angle;
  target_angle = current_angle;
  prev_micros = micros();
  enabled = false;
  last_output_pwm = 0;
  return true;
}

void set_target_from_command(int16_t signed_pwm) {
  if (signed_pwm > 255) signed_pwm = 255;
  if (signed_pwm < -255) signed_pwm = -255;

  int32_t delta;
  if (signed_pwm >= 0) {
    delta = (int32_t)signed_pwm * (int32_t)right_span / 255;
  } else {
    delta = (int32_t)(-signed_pwm) * (int32_t)left_span / 255;
  }
  target_angle = clamp_raw((int32_t)config::STEER_CENTER_RAW + delta);
}

void freeze_target_at_current() {
  target_angle = current_angle;
}

void enable() {
  enabled = true;
  digitalWrite(config::PIN_STEER_R_EN, HIGH);
  digitalWrite(config::PIN_STEER_L_EN, HIGH);
}

void disable() {
  enabled = false;
  analogWrite(config::PIN_STEER_RPWM, 0);
  analogWrite(config::PIN_STEER_LPWM, 0);
  digitalWrite(config::PIN_STEER_R_EN, LOW);
  digitalWrite(config::PIN_STEER_L_EN, LOW);
  last_output_pwm = 0;
}

void emergency_stop() {
  disable();
}

void update() {
  current_angle = read_angle();

  int16_t err = (int16_t)target_angle - (int16_t)current_angle;

  uint32_t now = micros();
  float dt_s = (now - prev_micros) * 1e-6f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 1.0f / config::CONTROL_LOOP_HZ;
  prev_micros = now;

  int16_t dmeas = (int16_t)current_angle - (int16_t)prev_measurement;
  prev_measurement = current_angle;
  float dmeas_per_sec = (float)dmeas / dt_s;

  float output_f = config::STEER_KP * (float)err - config::STEER_KD * dmeas_per_sec;

  if (abs(err) < config::STEER_DEADBAND_UNITS) {
    output_f = 0.0f;
  }

  const float max_pwm = (float)config::STEER_MAX_PWM;
  if (output_f > max_pwm) output_f = max_pwm;
  if (output_f < -max_pwm) output_f = -max_pwm;

  int16_t output = (int16_t)output_f;

  if (output != 0) {
    if (output > 0 && output < config::STEER_MIN_MOVE_PWM) output = config::STEER_MIN_MOVE_PWM;
    if (output < 0 && output > -(int16_t)config::STEER_MIN_MOVE_PWM) {
      output = -(int16_t)config::STEER_MIN_MOVE_PWM;
    }
  }

  int16_t cur_offset = (int16_t)current_angle - (int16_t)config::STEER_CENTER_RAW;
  int16_t upper_offset = max(left_span, right_span);
  int16_t lower_offset = min(left_span, right_span);
  if (output > 0 && cur_offset >= upper_offset) output = 0;
  if (output < 0 && cur_offset <= lower_offset) output = 0;

  drive_pwm(output);
}

uint16_t current_angle_raw() { return current_angle; }
uint16_t target_raw() { return target_angle; }
int16_t last_pwm() { return last_output_pwm; }

float normalized_position() {
  const int16_t offset = (int16_t)current_angle - (int16_t)config::STEER_CENTER_RAW;
  if (offset == 0) return 0.0f;
  if (left_span != 0 && (offset > 0) == (left_span > 0)) {
    float v = (float)offset / (float)left_span;
    if (v > 1.0f) v = 1.0f;
    return v;
  }
  if (right_span != 0) {
    float v = (float)offset / (float)right_span;
    if (v > 1.0f) v = 1.0f;
    return -v;
  }
  return 0.0f;
}

}
