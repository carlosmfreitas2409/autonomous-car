#include "ultrasonic.h"

#include <Arduino.h>

#include "config.h"

namespace ultrasonic {

void init() {
  for (uint8_t i = 0; i < config::SENSOR_COUNT; i++) {
    pinMode(config::PIN_TRIG[i], OUTPUT);
    digitalWrite(config::PIN_TRIG[i], LOW);
    pinMode(config::PIN_ECHO[i], INPUT);
  }
}

uint8_t read_cm(uint8_t index) {
  digitalWrite(config::PIN_TRIG[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(config::PIN_TRIG[index], LOW);

  const uint32_t width_us =
      pulseIn(config::PIN_ECHO[index], HIGH, config::ECHO_TIMEOUT_US);
  if (width_us == 0) return config::DIST_NONE;

  const uint32_t cm = width_us / 58;
  if (cm < 1 || cm > config::MAX_RANGE_CM) return config::DIST_NONE;
  return (uint8_t)cm;
}

}  // namespace ultrasonic
