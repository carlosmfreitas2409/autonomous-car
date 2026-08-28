#include "obstacle.h"

#include <Arduino.h>

#include "can_bus.h"
#include "config.h"

namespace obstacle {

namespace {

uint8_t distances[config::SONAR_SENSORS];
uint32_t last_frame_ms = 0;
bool ever_seen = false;

State current = State::STALE;
uint8_t nearest = config::SONAR_NO_ECHO;

void recompute() {
  const uint32_t now = millis();

  if (!ever_seen || (now - last_frame_ms) > config::SONAR_TIMEOUT_MS) {
    current = State::STALE;
    nearest = config::SONAR_NO_ECHO;
    return;
  }

  uint8_t closest = config::SONAR_NO_ECHO;
  for (uint8_t i = 0; i < config::SONAR_SENSORS; i++) {
    if (distances[i] < closest) closest = distances[i];
  }
  nearest = closest;

  if (closest <= config::STOP_CM) {
    current = State::BLOCKED;
  } else if (closest < config::SLOW_CM) {
    current = State::SLOWING;
  } else {
    current = State::CLEAR;
  }
}

}  // namespace

void init() {
  for (uint8_t i = 0; i < config::SONAR_SENSORS; i++) distances[i] = config::SONAR_NO_ECHO;
  last_frame_ms = millis();
}

void update() {
  uint8_t data[8];
  if (can_bus::poll_sonar(data)) {
    for (uint8_t i = 0; i < config::SONAR_SENSORS; i++) distances[i] = data[i];
    last_frame_ms = millis();
    ever_seen = true;
  }
  recompute();
}

uint8_t nearest_cm() {
  return nearest;
}

uint8_t distance_cm(uint8_t index) {
  return (index < config::SONAR_SENSORS) ? distances[index] : config::SONAR_NO_ECHO;
}

State state() {
  return current;
}

char state_char() {
  switch (current) {
    case State::CLEAR: return 'C';
    case State::SLOWING: return 'S';
    case State::BLOCKED: return 'B';
    case State::STALE: return 'X';
  }
  return '?';
}

}  // namespace obstacle
