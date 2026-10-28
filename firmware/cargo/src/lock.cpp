#include "lock.h"

#include <Arduino.h>

#include "config.h"

namespace lock {

namespace {

bool powered = false;
uint32_t on_since_ms = 0;
bool resting = false;
uint32_t rest_since_ms = 0;

void power(bool on) {
  digitalWrite(config::PIN_LOCK, on ? config::LOCK_ON : config::LOCK_OFF);
  powered = on;
}

}

void init() {
  digitalWrite(config::PIN_LOCK, config::LOCK_OFF);
  pinMode(config::PIN_LOCK, OUTPUT);
  power(false);
}

void update(bool requested, bool lid_closed) {
  const uint32_t now = millis();
  if (resting && (now - rest_since_ms) >= config::LOCK_REST_MS) resting = false;

  if (!requested || !lid_closed || resting) {
    if (powered) power(false);
    return;
  }

  if (!powered) {
    power(true);
    on_since_ms = now;
  } else if ((now - on_since_ms) >= config::LOCK_MAX_ON_MS) {
    power(false);
    resting = true;
    rest_since_ms = now;
  }
}

bool unlocked() {
  return powered;
}

}
