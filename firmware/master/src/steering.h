#pragma once

#include <stdint.h>

namespace steering {

enum class Fault : uint8_t {
  NONE = 0,
  POT_DISCONNECTED,
  POT_OUT_OF_RANGE,
  STALL,
};

bool init();

void set_target_from_command(int16_t signed_pwm);

void freeze_target_at_current();

void enable();
void disable();

void update();

void emergency_stop();

Fault fault();
char fault_char();

uint16_t current_angle_raw();
uint16_t target_raw();
int16_t last_pwm();

float normalized_position();

}  // namespace steering
