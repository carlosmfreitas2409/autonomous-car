#include "relays.h"

#include <Arduino.h>

#include "config.h"

namespace relays {

namespace {

Intent intent = {};

bool blink_phase_on = false;
uint32_t blink_phase_started_ms = 0;

void write_relay(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

}

void init() {
  const uint8_t pins[] = {
      config::PIN_RELAY_RIGHT_TURN, config::PIN_RELAY_LEFT_TURN,
      config::PIN_RELAY_DRL,        config::PIN_RELAY_LOW_BEAM,
      config::PIN_RELAY_BRAKE,      config::PIN_RELAY_REVERSE,
      config::PIN_RELAY_LOGO,
  };
  for (uint8_t i = 0; i < sizeof(pins); ++i) {
    pinMode(pins[i], OUTPUT);
    write_relay(pins[i], false);
  }
  intent = Intent{};
  blink_phase_on = false;
  blink_phase_started_ms = 0;
}

void set_intent(const Intent& next) {
  intent = next;
}

void all_off() {
  intent = Intent{};
}

void update(uint32_t now_ms) {
  write_relay(config::PIN_RELAY_DRL, intent.drl);
  write_relay(config::PIN_RELAY_LOW_BEAM, intent.low_beam);
  write_relay(config::PIN_RELAY_BRAKE, intent.brake);
  write_relay(config::PIN_RELAY_REVERSE, intent.reverse);
  write_relay(config::PIN_RELAY_LOGO, intent.logo);

  const bool any_signal = intent.left_turn || intent.right_turn;
  if (!any_signal) {
    blink_phase_on = false;
    blink_phase_started_ms = now_ms;
    write_relay(config::PIN_RELAY_LEFT_TURN, false);
    write_relay(config::PIN_RELAY_RIGHT_TURN, false);
    return;
  }

  const uint16_t phase_dur =
      blink_phase_on ? config::BLINK_ON_MS : config::BLINK_OFF_MS;
  if ((now_ms - blink_phase_started_ms) >= phase_dur) {
    blink_phase_on = !blink_phase_on;
    blink_phase_started_ms = now_ms;
  }

  write_relay(config::PIN_RELAY_LEFT_TURN,  intent.left_turn  && blink_phase_on);
  write_relay(config::PIN_RELAY_RIGHT_TURN, intent.right_turn && blink_phase_on);
}

}
