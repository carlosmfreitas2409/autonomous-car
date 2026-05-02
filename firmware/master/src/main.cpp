#include <Arduino.h>

#include "command_parser.h"
#include "config.h"
#include "motors.h"
#include "safety.h"
#include "steering.h"
#include "telemetry.h"

namespace {

// Bumps PWM frequency from Arduino's default 490 Hz to ~31 kHz, above audible
// range so motors don't whine. Pokes AVR timer registers directly because there
// is no Arduino API for changing PWM frequency.
void configure_high_freq_pwm() {
  // Each block: phase-correct 8-bit mode + prescaler 1 → ~31 kHz.
  // Timer 0 (millis/micros) and Timer 2 (tone) are intentionally untouched.
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM40);
  TCCR4B = _BV(CS40);
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM50);
  TCCR5B = _BV(CS50);
}

uint32_t last_control_ms = 0;
bool fault_blink_state = false;
uint32_t last_blink_ms = 0;

}

void setup() {
  Serial.begin(config::SERIAL_BAUD);

  pinMode(config::PIN_STATUS_LED, OUTPUT);
  digitalWrite(config::PIN_STATUS_LED, LOW);

  configure_high_freq_pwm();

  motors::init();

  bool sensor_ok = steering::init();

  safety::init();

  Serial.println(F("RDY car-control v1"));

  if (!sensor_ok) {
    Serial.println(F("ERR AS5600"));
    safety::enter_fault();
  }

  last_control_ms = millis();
}

void loop() {
  command_parser::poll();
  safety::update();

  const uint32_t now = millis();
  if ((now - last_control_ms) >= config::CONTROL_LOOP_PERIOD_MS) {
    last_control_ms = now;
    motors::update();
    if (safety::state() != safety::State::FAULT) {
      steering::update();
    }
  }

  telemetry::update();

  const uint32_t blink_period = (safety::state() == safety::State::FAULT) ? 100 : 1000;
  if ((now - last_blink_ms) >= blink_period) {
    last_blink_ms = now;
    if (safety::state() == safety::State::FAULT) {
      fault_blink_state = !fault_blink_state;
      digitalWrite(config::PIN_STATUS_LED, fault_blink_state ? HIGH : LOW);
    } else if (safety::state() == safety::State::ESTOP) {
      digitalWrite(config::PIN_STATUS_LED, LOW);
    } else {
      digitalWrite(config::PIN_STATUS_LED, HIGH);
    }
  }
}
