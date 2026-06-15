#include "safety.h"

#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "steering.h"

namespace safety {

namespace {

State current_state = State::ESTOP;
uint32_t last_command_ms = 0;

}  // namespace

void init() {
  current_state = State::ESTOP;
  last_command_ms = millis();
  motors::disable();
  steering::disable();
}

void note_command() {
  last_command_ms = millis();
  if (current_state == State::WATCHDOG_IDLE) {
    current_state = State::RUN;
    motors::enable();
    steering::enable();
  }
}

void update() {
  if (current_state == State::ESTOP || current_state == State::FAULT) return;

  const uint32_t elapsed = millis() - last_command_ms;
  if (elapsed > config::WATCHDOG_TIMEOUT_MS && current_state == State::RUN) {
    current_state = State::WATCHDOG_IDLE;
    motors::set_throttle(0);
    // Freeze target at the current angle so wheels stop where they are
    // instead of snapping back to center.
    steering::freeze_target_at_current();
  }
}

void trigger_estop() {
  current_state = State::ESTOP;
  motors::emergency_stop();
  steering::emergency_stop();
}

void clear_estop_via_motor_command(int16_t throttle, int16_t steering) {
  if (current_state != State::ESTOP) return;
  if (throttle == 0 && steering == 0) return;
  current_state = State::RUN;
  motors::enable();
  steering::enable();
}

void enter_fault() {
  current_state = State::FAULT;
  motors::emergency_stop();
  steering::emergency_stop();
}

State state() {
  return current_state;
}

char state_char() {
  switch (current_state) {
    case State::RUN: return 'R';
    case State::WATCHDOG_IDLE: return 'W';
    case State::ESTOP: return 'E';
    case State::FAULT: return 'F';
  }
  return '?';
}

uint16_t ms_since_last_command() {
  const uint32_t elapsed = millis() - last_command_ms;
  return elapsed > 0xFFFF ? 0xFFFF : (uint16_t)elapsed;
}

}  // namespace safety
