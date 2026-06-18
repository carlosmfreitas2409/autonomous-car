#pragma once

#include <stdint.h>

namespace steering {

bool init();

void set_target_from_command(int16_t signed_pwm);

void freeze_target_at_current();

void enable();
void disable();

void update();

void emergency_stop();

uint16_t current_angle_raw();
uint16_t target_raw();
int16_t last_pwm();

// Measured steering position normalized to [-1, 1]: positive = steered toward
// STEER_MAX_LEFT_RAW (left), negative = toward STEER_MAX_RIGHT_RAW (right),
// 0 = centered. Used by the drive electronic differential.
float normalized_position();

}  // namespace steering
