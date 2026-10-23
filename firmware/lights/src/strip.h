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
  CYAN,        // SAE J3134 turquoise (strip.cpp), the id kept for the protocol
  RED_PULSE,
  POLICE,
  BLUE_BREATHE,
  CYAN_SWEEP,
  YELLOW_PULSE,
};

void init();

void set_effect(uint8_t id);

void off();

void update(uint32_t now_ms);

}
