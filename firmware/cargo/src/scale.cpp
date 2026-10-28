#include "scale.h"

#include <Arduino.h>
#include <HX711.h>

#include "config.h"

namespace scale {

namespace {

HX711 hx711;
bool tared = false;
uint32_t last_reading_ms = 0;

float readings[config::SCALE_STABLE_WINDOW];
uint8_t count = 0;
uint8_t next = 0;

float reading(uint8_t age) {
  return readings[(next + config::SCALE_STABLE_WINDOW - 1 - age) % config::SCALE_STABLE_WINDOW];
}

}

bool init() {
  hx711.begin(config::PIN_SCALE_DT, config::PIN_SCALE_SCK);
  hx711.set_scale(config::SCALE_COUNTS_PER_GRAM);
  if (!hx711.wait_ready_timeout(config::SCALE_READY_TIMEOUT_MS)) return false;
  hx711.tare(config::SCALE_TARE_SAMPLES);
  tared = true;
  last_reading_ms = millis();
  return true;
}

void update() {
  if (!tared || !hx711.is_ready()) return;
  readings[next] = hx711.get_units(1);
  next = (next + 1) % config::SCALE_STABLE_WINDOW;
  if (count < config::SCALE_STABLE_WINDOW) count++;
  last_reading_ms = millis();
}

int16_t grams() {
  if (fault() || count == 0) return 0;
  const uint8_t n = count < config::SCALE_AVERAGE ? count : config::SCALE_AVERAGE;
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) sum += reading(i);
  const float mean = sum / n;
  if (mean > INT16_MAX) return INT16_MAX;
  if (mean < INT16_MIN) return INT16_MIN;
  return (int16_t)lroundf(mean);
}

bool stable() {
  if (fault() || count < config::SCALE_STABLE_WINDOW) return false;
  float low = reading(0);
  float high = low;
  for (uint8_t i = 1; i < count; i++) {
    low = min(low, reading(i));
    high = max(high, reading(i));
  }
  return (high - low) <= config::SCALE_STABLE_SPAN_G;
}

bool fault() {
  return !tared || (millis() - last_reading_ms) > config::SCALE_FAULT_MS;
}

}
