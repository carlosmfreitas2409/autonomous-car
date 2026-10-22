#include <Arduino.h>

#include "config.h"
#include "lid.h"
#include "lock.h"
#include "scale.h"
#include "sonar_link.h"

namespace {

uint32_t last_send_ms = 0;
uint32_t last_debug_ms = 0;

}  // namespace

void setup() {
  // Locked before anything else: a low pin is an open lock.
  lock::init();

  Serial.begin(config::SERIAL_BAUD);

  lid::init();
  sonar_link::init();
  const bool scale_ok = scale::init();

  Serial.println(F("RDY car-cargo v1"));
  if (!scale_ok) Serial.println(F("ERR SCALE"));
}

void loop() {
  const uint32_t now = millis();

  sonar_link::update();
  lid::update();
  scale::update();
  lock::update(sonar_link::unlock_requested(), lid::closed());

  if ((now - last_send_ms) >= config::STATUS_PERIOD_MS) {
    last_send_ms = now;
    sonar_link::send_status(scale::grams(), scale::stable(), lid::closed(), lock::unlocked(), scale::fault());
  }

  // The same line on this node's own USB, for a look without the sonar node.
  if ((now - last_debug_ms) >= config::DEBUG_PERIOD_MS) {
    last_debug_ms = now;
    Serial.printf("C %d %d %d %d %d\n", scale::grams(), scale::stable(), lid::closed(),
                  lock::unlocked(), scale::fault());
  }
}
