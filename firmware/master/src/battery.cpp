#include "battery.h"

#include <Arduino.h>

#include "config.h"

namespace battery {

namespace {

float filtered_pct = 0.0f;
float filtered_volts = 0.0f;
uint32_t last_sample_ms = 0;
bool led_on[config::BATT_LED_COUNT] = {};

float read_battery_volts() {
  analogRead(config::PIN_BATT_SENSE);
  uint16_t sum = 0;
  for (uint8_t i = 0; i < config::BATT_ADC_SAMPLES; i++) {
    sum += (uint16_t)analogRead(config::PIN_BATT_SENSE);
  }
  return (sum / (float)config::BATT_ADC_SAMPLES) * config::BATT_VOLTS_PER_COUNT;
}

float volts_to_pct(float volts) {
  constexpr uint8_t last = config::BATT_CURVE_POINTS - 1;
  if (volts <= config::BATT_CURVE_V[0]) return config::BATT_CURVE_PCT[0];
  if (volts >= config::BATT_CURVE_V[last]) return config::BATT_CURVE_PCT[last];
  for (uint8_t i = 1; i <= last; i++) {
    if (volts < config::BATT_CURVE_V[i]) {
      const float v0 = config::BATT_CURVE_V[i - 1];
      const float p0 = config::BATT_CURVE_PCT[i - 1];
      const float t = (volts - v0) / (config::BATT_CURVE_V[i] - v0);
      return p0 + t * (config::BATT_CURVE_PCT[i] - p0);
    }
  }
  return config::BATT_CURVE_PCT[last];
}

void refresh_leds() {
  for (uint8_t i = 0; i < config::BATT_LED_COUNT; i++) {
    const float threshold = config::BATT_LED_THRESHOLD_PCT[i];
    if (led_on[i]) {
      led_on[i] = filtered_pct >= threshold - config::BATT_LED_HYSTERESIS_PCT;
    } else {
      led_on[i] = filtered_pct >= threshold;
    }
    digitalWrite(config::BATT_LED_PINS[i], led_on[i] ? HIGH : LOW);
  }
}

}

void init() {
  for (uint8_t i = 0; i < config::BATT_LED_COUNT; i++) {
    pinMode(config::BATT_LED_PINS[i], OUTPUT);
    digitalWrite(config::BATT_LED_PINS[i], LOW);
  }
  filtered_volts = read_battery_volts();
  filtered_pct = volts_to_pct(filtered_volts);
  refresh_leds();
  last_sample_ms = millis();
}

void update() {
  const uint32_t now = millis();
  if (now - last_sample_ms < config::BATT_SAMPLE_PERIOD_MS) return;
  last_sample_ms = now;

  const float volts_now = read_battery_volts();
  filtered_volts += config::BATT_FILTER_ALPHA * (volts_now - filtered_volts);
  filtered_pct += config::BATT_FILTER_ALPHA * (volts_to_pct(volts_now) - filtered_pct);
  refresh_leds();
}

float volts() { return filtered_volts; }

float percent() { return filtered_pct; }

}
