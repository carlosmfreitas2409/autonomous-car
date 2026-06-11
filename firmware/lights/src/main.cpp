#include <Arduino.h>

#include "can_bus.h"
#include "config.h"
#include "relays.h"

namespace {

uint32_t last_frame_ms = 0;
bool watchdog_tripped = true;

void apply_bitmask(uint8_t mask) {
  relays::Intent intent{};
  intent.left_turn  = (mask & (1u << 0)) != 0;
  intent.right_turn = (mask & (1u << 1)) != 0;
  intent.drl        = (mask & (1u << 2)) != 0;
  intent.low_beam   = (mask & (1u << 3)) != 0;
  intent.brake      = (mask & (1u << 4)) != 0;
  intent.reverse    = (mask & (1u << 5)) != 0;
  intent.logo       = (mask & (1u << 6)) != 0;
  relays::set_intent(intent);
}

}

void setup() {
  Serial.begin(config::SERIAL_BAUD);

  pinMode(config::PIN_STATUS_LED, OUTPUT);
  digitalWrite(config::PIN_STATUS_LED, HIGH);

  relays::init();

  const bool can_ok = can_bus::init();

  Serial.println(F("RDY car-lights"));
  if (!can_ok) Serial.println(F("ERR CAN"));
}

void loop() {
  const uint32_t now = millis();

  uint8_t mask = 0;
  if (can_bus::poll(mask)) {
    last_frame_ms = now;
    watchdog_tripped = false;
    apply_bitmask(mask);
  } else if (!watchdog_tripped && (now - last_frame_ms) >= config::CAN_WATCHDOG_MS) {
    watchdog_tripped = true;
    relays::all_off();
  }

  relays::update(now);
}
