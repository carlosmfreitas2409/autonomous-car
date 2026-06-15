#pragma once

#include <stdint.h>

namespace motors {

void init();

void set_throttle(int16_t throttle);

int16_t current_throttle();

void enable();
void disable();

void update();

void emergency_stop();

}  // namespace motors
