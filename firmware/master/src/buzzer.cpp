#include "buzzer.h"

#include <Arduino.h>

#include "config.h"

namespace buzzer {

namespace {

bool beeping = false;
uint32_t phase_start_ms = 0;

void silence() {
  noTone(config::PIN_BUZZER);
  digitalWrite(config::PIN_BUZZER, LOW);
  beeping = false;
}

}

void init() {
  pinMode(config::PIN_BUZZER, OUTPUT);
  silence();
  phase_start_ms = millis() - config::BUZZER_BEEP_OFF_MS;
}

void update(bool active) {
  const uint32_t now = millis();

  if (!active) {
    if (beeping) silence();
    phase_start_ms = now - config::BUZZER_BEEP_OFF_MS;
    return;
  }

  const uint32_t elapsed = now - phase_start_ms;
  if (beeping) {
    if (elapsed >= config::BUZZER_BEEP_ON_MS) {
      noTone(config::PIN_BUZZER);
      beeping = false;
      phase_start_ms = now;
    }
  } else {
    if (elapsed >= config::BUZZER_BEEP_OFF_MS) {
      tone(config::PIN_BUZZER, config::BUZZER_TONE_HZ);
      beeping = true;
      phase_start_ms = now;
    }
  }
}

}
