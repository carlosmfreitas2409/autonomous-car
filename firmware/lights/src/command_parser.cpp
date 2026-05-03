#include "command_parser.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "relays.h"

namespace command_parser {

namespace {

constexpr uint8_t LINE_BUFFER_SIZE = 32;

char line_buf[LINE_BUFFER_SIZE];
uint8_t line_len = 0;

bool parse_bool_token(char*& save, bool& out) {
  char* tok = strtok_r(nullptr, " \t", &save);
  if (!tok) return false;
  const long v = atol(tok);
  out = (v != 0);
  return true;
}

void handle_lights(char* args) {
  char* save = nullptr;

  char* first = strtok_r(args, " \t", &save);
  if (!first) {
    Serial.println(F("ERR L"));
    return;
  }

  relays::Intent next{};
  next.left_turn = (atol(first) != 0);
  if (!parse_bool_token(save, next.right_turn) ||
      !parse_bool_token(save, next.drl) ||
      !parse_bool_token(save, next.low_beam) ||
      !parse_bool_token(save, next.brake) ||
      !parse_bool_token(save, next.reverse) ||
      !parse_bool_token(save, next.logo)) {
    Serial.println(F("ERR L"));
    return;
  }

  relays::set_intent(next);
}

void dispatch(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == 0) return;
  const char verb = *line;
  char* args = line + 1;
  while (*args == ' ' || *args == '\t') ++args;

  switch (verb) {
    case 'L': handle_lights(args); break;
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
