#pragma once

#include <stdint.h>

namespace relays {

struct Intent {
  bool left_turn;
  bool right_turn;
  bool drl;
  bool low_beam;
  bool brake;
  bool reverse;
  bool logo;
  bool projector;
};

void init();

void set_intent(const Intent& intent);

void all_off();

void update(uint32_t now_ms);

}
