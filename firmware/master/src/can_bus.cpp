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
  SPI.begin();

  if (mcp.reset() != MCP2515::ERROR_OK) {
    ready = false;
    return false;
  }

  if (mcp.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    ready = false;
    return false;
  }

  if (mcp.setNormalMode() != MCP2515::ERROR_OK) {
    ready = false;
    return false;
  }

  ready = true;
  return true;
}

bool send_lights(uint8_t bitmask, uint8_t effect) {
  if (!ready) return false;
  struct can_frame frame;
  frame.can_id = config::CAN_LIGHTS_ID;
  frame.can_dlc = 2;
  frame.data[0] = bitmask;
  frame.data[1] = effect;
  return mcp.sendMessage(&frame) == MCP2515::ERROR_OK;
}

}
