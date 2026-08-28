#pragma once

#include <stdint.h>

namespace obstacle {

enum class State : uint8_t {
  CLEAR = 0,
  SLOWING,
  BLOCKED,
  STALE,
};

void init();
void update();

uint8_t nearest_cm();
uint8_t distance_cm(uint8_t index);

State state();
char state_char();

}  // namespace obstacle
