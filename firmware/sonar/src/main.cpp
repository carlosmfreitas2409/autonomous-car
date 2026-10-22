#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "cargo_bridge.h"
#include "config.h"
#include "ultrasonic.h"

namespace {

MCP2515 mcp(config::PIN_CAN_CS);
uint32_t last_send_ms = 0;

void send_distances() {
  struct can_frame frame;
  frame.can_id = config::CAN_SONAR_ID;
  frame.can_dlc = config::SENSOR_COUNT;
  for (uint8_t i = 0; i < config::SENSOR_COUNT; i++) {
    if (i > 0) delay(config::PING_SETTLE_MS);
    frame.data[i] = ultrasonic::read_cm(i);
  }
  mcp.sendMessage(&frame);
}

}  // namespace

void setup() {
  Serial.begin(config::SERIAL_BAUD);
  pinMode(config::PIN_STATUS_LED, OUTPUT);

  ultrasonic::init();

  SPI.begin();
  mcp.reset();
  mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
  cargo_bridge::init(mcp);
  mcp.setNormalMode();

  Serial.println(F("RDY car-sonar v1"));
}

void loop() {
  cargo_bridge::update();

  const uint32_t now = millis();
  if ((now - last_send_ms) < config::CAN_PERIOD_MS) return;
  last_send_ms = now;

  send_distances();
  digitalWrite(config::PIN_STATUS_LED, (now / 1000) & 1);
}
