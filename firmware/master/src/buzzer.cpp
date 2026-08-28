#include "buzzer.h"

#include <Arduino.h>

#include "config.h"

namespace buzzer {

namespace {

bool beeping = false;
uint32_t phase_start_ms = 0;

constexpr uint16_t kOcrTop =
    (uint16_t)(F_CPU / (2UL * 8UL * config::BUZZER_TONE_HZ) - 1UL);

void tone_on() {
  TCCR5A = _BV(COM5A0);
  TCCR5B = _BV(WGM52) | _BV(CS51);
  OCR5A = kOcrTop;
  TCNT5 = 0;
}

void tone_off() {
  TCCR5A = 0;
  TCCR5B = 0;
  digitalWrite(config::PIN_BUZZER, LOW);
}

void silence() {
  tone_off();
  beeping = false;
}

}  // namespace

void init() {
  pinMode(config::PIN_BUZZER, OUTPUT);
  silence();
  phase_start_ms = millis() - config::BUZZER_BEEP_OFF_MS;
}

void update(bool reversing, bool alarm) {
  const uint32_t now = millis();

  if (alarm) {
    if (!beeping) {
      tone_on();
      beeping = true;
    }
    phase_start_ms = now;
    return;
  }

  if (!reversing) {
    if (beeping) silence();
    phase_start_ms = now - config::BUZZER_BEEP_OFF_MS;
    return;
  }

  const uint32_t elapsed = now - phase_start_ms;
  if (beeping) {
    if (elapsed >= config::BUZZER_BEEP_ON_MS) {
      tone_off();
      beeping = false;
      phase_start_ms = now;
    }
  } else {
    if (elapsed >= config::BUZZER_BEEP_OFF_MS) {
      tone_on();
      beeping = true;
      phase_start_ms = now;
    }
  }
}

}  // namespace buzzer
