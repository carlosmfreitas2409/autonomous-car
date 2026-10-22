#include "lid.h"

#include <Arduino.h>

#include "config.h"

namespace lid {

namespace {

bool stable_closed = false;
bool last_raw = false;
uint32_t last_change_ms = 0;

}

void init() {
  // External pull-down: no internal pull.
  pinMode(config::PIN_LID, INPUT);
  last_raw = stable_closed = digitalRead(config::PIN_LID) == HIGH;
  last_change_ms = millis();
}

void update() {
  const uint32_t now = millis();
  const bool raw = digitalRead(config::PIN_LID) == HIGH;
  if (raw != last_raw) {
    last_raw = raw;
    last_change_ms = now;
  }
  if ((now - last_change_ms) >= config::LID_DEBOUNCE_MS) stable_closed = raw;
}

bool closed() {
  return stable_closed;
}

}  // namespace lid
