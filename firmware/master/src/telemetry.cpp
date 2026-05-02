#include "telemetry.h"

#ifdef TELEMETRY_ENABLED

#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "safety.h"
#include "steering.h"

namespace telemetry {

namespace {
uint32_t last_emit_ms = 0;
}

void update() {
  const uint32_t now = millis();
  if (now - last_emit_ms < config::TELEMETRY_PERIOD_MS) return;
  last_emit_ms = now;

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
}

}  // namespace telemetry

#else

namespace telemetry {
void update() {}
}  // namespace telemetry

#endif
