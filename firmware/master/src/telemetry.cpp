#include "telemetry.h"

#include <Arduino.h>

#include "config.h"

#ifdef TELEMETRY_ENABLED
#include "battery.h"
#include "motors.h"
#include "obstacle.h"
#include "safety.h"
#include "steering.h"

namespace {
uint32_t last_emit_ms = 0;
uint32_t last_battery_ms = 0;
}
#endif

namespace telemetry {

void update() {
#ifdef TELEMETRY_ENABLED
  const uint32_t now = millis();
  if (now - last_emit_ms < config::TELEMETRY_PERIOD_MS) return;
  last_emit_ms = now;

  Serial.print(F("S "));
  Serial.print(obstacle::state_char());
  Serial.print(' ');
  Serial.print(obstacle::nearest_cm());
  for (uint8_t i = 0; i < config::SONAR_SENSORS; i++) {
    Serial.print(' ');
    Serial.print(obstacle::distance_cm(i));
  }
  Serial.println();

  Serial.print(F("T "));
  Serial.print(steering::current_angle_raw());
  Serial.print(' ');
  Serial.print(steering::target_raw());
  Serial.print(' ');
  Serial.print(motors::current_throttle());
  Serial.print(' ');
  Serial.print(steering::last_pwm());
  Serial.print(' ');
  Serial.print(safety::state_char());
  Serial.print(' ');
  Serial.println(safety::ms_since_last_command());

  if (now - last_battery_ms >= config::BATT_TELEMETRY_PERIOD_MS) {
    last_battery_ms = now;
    Serial.print(F("B "));
    Serial.print((unsigned long)(battery::volts() * 1000.0f + 0.5f));
    Serial.print(' ');
    Serial.println((int)(battery::percent() + 0.5f));
  }
#endif
}

}
