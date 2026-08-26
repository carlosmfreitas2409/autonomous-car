#include "can_bus.h"

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"

namespace can_bus {

namespace {

MCP2515 mcp(config::PIN_CAN_CS);
bool ready = false;

}

bool init() {
  pinMode(config::PIN_CAN_INT, INPUT);
  SPI.begin();
  
  if (mcp.reset() != MCP2515::ERROR_OK) return false;
  if (mcp.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) return false;
  if (mcp.setNormalMode() != MCP2515::ERROR_OK) return false;

  ready = true;
  return true;
}

bool poll(uint8_t& bitmask, uint8_t& effect) {
  if (!ready) return false;

  if (digitalRead(config::PIN_CAN_INT) != LOW) return false;

  struct can_frame frame;
  if (mcp.readMessage(&frame) != MCP2515::ERROR_OK) return false;

  if (frame.can_id != config::CAN_LIGHTS_ID || frame.can_dlc < 1) return false;

  bitmask = frame.data[0];
  effect = (frame.can_dlc >= 2) ? frame.data[1] : 0;

  return true;
}

}
