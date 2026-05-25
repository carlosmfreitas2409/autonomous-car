#include "steering.h"

#include <AS5600.h>
#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace steering {

namespace {

AS5600 sensor;

int16_t left_span = 0;
int16_t right_span = 0;

uint16_t target_angle = config::STEER_CENTER_RAW;
uint16_t current_angle = config::STEER_CENTER_RAW;

bool enabled = false;
int16_t last_output_pwm = 0;

uint16_t prev_measurement = config::STEER_CENTER_RAW;
uint32_t prev_micros = 0;

// Shortest signed angular distance on a 4096-unit circle (AS5600 wraps at 0/4095).
int16_t wrap_diff_4096(uint16_t a, uint16_t b) {
  int16_t d = (int16_t)a - (int16_t)b;
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

// Folds any int back into the 0..4095 range — the AS5600's native domain.
uint16_t wrap_4096(int32_t v) {
  v %= 4096;
  if (v < 0) v += 4096;
  return (uint16_t)v;
}

void drive_pwm(int16_t signed_pwm) {
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
  uint16_t a = sensor.rawAngle();
  return a & 0x0FFF;
}

}  // namespace

bool init() {
  Wire.begin();
  Wire.setClock(400000);
  sensor.begin();

  if (!sensor.isConnected()) return false;
  if (!sensor.magnetDetected()) return false;

  pinMode(config::PIN_STEER_R_EN, OUTPUT);
  pinMode(config::PIN_STEER_L_EN, OUTPUT);
  pinMode(config::PIN_STEER_RPWM, OUTPUT);
  pinMode(config::PIN_STEER_LPWM, OUTPUT);
  
  digitalWrite(config::PIN_STEER_R_EN, LOW);
  digitalWrite(config::PIN_STEER_L_EN, LOW);
  analogWrite(config::PIN_STEER_RPWM, 0);
  analogWrite(config::PIN_STEER_LPWM, 0);

  left_span = wrap_diff_4096(config::STEER_MAX_LEFT_RAW, config::STEER_CENTER_RAW);
  right_span = wrap_diff_4096(config::STEER_MAX_RIGHT_RAW, config::STEER_CENTER_RAW);

  current_angle = read_angle();
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
  target_angle = wrap_4096((int32_t)config::STEER_CENTER_RAW + delta);
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

  int16_t err = wrap_diff_4096(target_angle, current_angle);

  uint32_t now = micros();
  float dt_s = (now - prev_micros) * 1e-6f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 1.0f / config::CONTROL_LOOP_HZ;
  prev_micros = now;

  // Derivative on measurement, not error: avoids a derivative kick when the host
  // jumps the setpoint (e.g. M 0 -255 → M 0 255).
  int16_t dmeas = wrap_diff_4096(current_angle, prev_measurement);
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

  // Below ~60 PWM the steering motor can't break static friction; bump small
  // non-zero outputs up so we either move decisively or not at all.
  if (output != 0) {
    if (output > 0 && output < config::STEER_MIN_MOVE_PWM) output = config::STEER_MIN_MOVE_PWM;
    if (output < 0 && output > -(int16_t)config::STEER_MIN_MOVE_PWM) {
      output = -(int16_t)config::STEER_MIN_MOVE_PWM;
    }
  }

  // Soft limits: refuse to drive past the configured mechanical stop, but
  // always allow recovery in the opposite direction.
  int16_t cur_offset = wrap_diff_4096(current_angle, config::STEER_CENTER_RAW);
  if (output > 0 && cur_offset >= right_span) output = 0;
  if (output < 0 && cur_offset <= left_span) output = 0;

  drive_pwm(output);
}

uint16_t current_angle_raw() { return current_angle; }
uint16_t target_raw() { return target_angle; }
int16_t last_pwm() { return last_output_pwm; }

}  // namespace steering
