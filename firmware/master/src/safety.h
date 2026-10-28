#pragma once

#include <stdint.h>

namespace safety {

enum class State : uint8_t {
  ESTOP = 0,
  RUN,
  WATCHDOG_IDLE,
  FAULT,
};

void init();

void note_command();

void update();

void trigger_estop();

void clear_estop_via_motor_command(int16_t throttle, int16_t steering);

void enter_fault();

State state();
char state_char();
uint16_t ms_since_last_command();

}
