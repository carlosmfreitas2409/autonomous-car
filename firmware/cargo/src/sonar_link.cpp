#include "sonar_link.h"

#include <Arduino.h>
#include <string.h>

#include "config.h"

namespace sonar_link {

namespace {

constexpr uint8_t LINE_BUFFER_SIZE = 16;

char line_buf[LINE_BUFFER_SIZE];
uint8_t line_len = 0;

bool unlock_seen = false;
uint32_t last_command_ms = 0;

void dispatch() {
  // Only the two exact commands: a garbled byte must never read as an unlock.
  if (strcmp(line_buf, "K 1") == 0) {
    unlock_seen = true;
  } else if (strcmp(line_buf, "K 0") == 0) {
    unlock_seen = false;  // an explicit lock ends the lease at once
  } else {
    return;
  }
  last_command_ms = millis();
}

}

void init() {
  Serial2.begin(config::LINK_BAUD, SERIAL_8N1, config::PIN_LINK_RX, config::PIN_LINK_TX);
}

void update() {
  while (Serial2.available()) {
    const int c = Serial2.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      line_buf[line_len] = 0;
      if (line_len > 0) dispatch();
      line_len = 0;
    } else if (line_len < LINE_BUFFER_SIZE - 1) {
      line_buf[line_len++] = (char)c;
    } else {
      line_len = 0;
    }
  }
}

void send_status(int16_t grams, bool stable, bool lid_closed, bool unlocked, bool fault) {
  Serial2.printf("C %d %d %d %d %d\n", grams, stable, lid_closed, unlocked, fault);
}

bool unlock_requested() {
  return unlock_seen && (millis() - last_command_ms) <= config::UNLOCK_LEASE_MS;
}

}  // namespace sonar_link
