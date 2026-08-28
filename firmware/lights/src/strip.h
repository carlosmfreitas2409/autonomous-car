#pragma once

#include <stdint.h>

namespace strip {

enum class Effect : uint8_t {
  OFF = 0,
  RED,
  GREEN,
  BLUE,
  STROBE,
  RAINBOW,
  CYAN,
  RED_PULSE,
  RED_STROBE, // RESERVED
};

void init();

void set_effect(uint8_t id);

void off();

void update(uint32_t now_ms);

}
