#pragma once

#include <stdint.h>

namespace motors {

void init();

void set_throttle(uint8_t throttle);

uint8_t current_throttle();

void enable();
void disable();

void update();

void emergency_stop();

}  // namespace motors
