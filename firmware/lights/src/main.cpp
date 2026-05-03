#include <Arduino.h>

#include "command_parser.h"
#include "config.h"
#include "relays.h"

void setup() {
  Serial.begin(config::SERIAL_BAUD);

  pinMode(config::PIN_STATUS_LED, OUTPUT);
  digitalWrite(config::PIN_STATUS_LED, HIGH);

  relays::init();

  Serial.println(F("RDY car-lights v1"));
}

void loop() {
  command_parser::poll();
  relays::update(millis());
}
