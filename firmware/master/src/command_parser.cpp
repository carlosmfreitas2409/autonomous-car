#include "command_parser.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "can_bus.h"
#include "motors.h"
#include "safety.h"
#include "steering.h"

namespace command_parser {

namespace {

constexpr uint8_t LINE_BUFFER_SIZE = 48;

char line_buf[LINE_BUFFER_SIZE];
uint8_t line_len = 0;

void handle_motor(char* args) {
  char* save = nullptr;
  char* tok_t = strtok_r(args, " \t", &save);
  char* tok_s = strtok_r(nullptr, " \t", &save);
  if (!tok_t || !tok_s) {
    Serial.println(F("ERR M"));
    return;
  }
  
  long t = atol(tok_t);
  long s = atol(tok_s);
  if (t < -255) t = -255;
  if (t > 255) t = 255;
  if (s < -255) s = -255;
  if (s > 255) s = 255;

  if (safety::state() == safety::State::FAULT) return;

  safety::clear_estop_via_motor_command((int16_t)t, (int16_t)s);

  if (safety::state() == safety::State::ESTOP) return;

  motors::set_throttle((int16_t)t);
  steering::set_target_from_command((int16_t)s);
  safety::note_command();
}

void handle_estop() {
  safety::trigger_estop();
  safety::note_command();
}

void handle_heartbeat() {
  safety::note_command();
}

bool parse_bit_token(char*& save, uint8_t bit, uint8_t& mask) {
  char* tok = strtok_r(nullptr, " \t", &save);
  if (!tok) return false;
  if (atol(tok) != 0) mask |= (uint8_t)(1u << bit);
  return true;
}

void handle_lights(char* args) {
  char* save = nullptr;
  char* first = strtok_r(args, " \t", &save);
  if (!first) {
    Serial.println(F("ERR L"));
    return;
  }

  uint8_t mask = 0;
  if (atol(first) != 0) mask |= (uint8_t)(1u << 0);
  if (!parse_bit_token(save, 1, mask) ||
      !parse_bit_token(save, 2, mask) ||
      !parse_bit_token(save, 3, mask) ||
      !parse_bit_token(save, 4, mask) ||
      !parse_bit_token(save, 5, mask) ||
      !parse_bit_token(save, 6, mask) ||
      !parse_bit_token(save, 7, mask)) {
    Serial.println(F("ERR L"));
    return;
  }

  uint8_t effect = 0;
  char* tok_effect = strtok_r(nullptr, " \t", &save);
  if (tok_effect) effect = (uint8_t)atol(tok_effect);

  if (!can_bus::send_lights(mask, effect)) {
    Serial.println(F("ERR CAN"));
  }
}

void handle_cargo(char* args) {
  char* save = nullptr;
  char* tok = strtok_r(args, " \t", &save);
  if (!tok) {
    Serial.println(F("ERR K"));
    return;
  }

  const bool unlock = atol(tok) != 0 && motors::current_throttle() == 0;
  if (!can_bus::send_cargo(unlock ? 1 : 0)) {
    Serial.println(F("ERR CAN"));
  }
}

void dispatch(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == 0) return;
  const char verb = *line;
  char* args = line + 1;
  while (*args == ' ' || *args == '\t') ++args;

  switch (verb) {
    case 'M': handle_motor(args); break;
    case 'E': handle_estop(); break;
    case 'H': handle_heartbeat(); break;
    case 'L': handle_lights(args); break;
    case 'K': handle_cargo(args); break;
    default:
      Serial.print(F("ERR "));
      Serial.println(verb);
      break;
  }
}

}

void poll() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      line_buf[line_len] = 0;
      if (line_len > 0) dispatch(line_buf);
      line_len = 0;
    } else if (line_len < LINE_BUFFER_SIZE - 1) {
      line_buf[line_len++] = (char)c;
    } else {
      line_len = 0;
    }
  }
}

}
