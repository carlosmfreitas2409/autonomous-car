#include "cargo_bridge.h"

#include <Arduino.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

namespace cargo_bridge {

namespace {

constexpr uint8_t LINE_BUFFER_SIZE = 32;

MCP2515* bus = nullptr;
char line_buf[LINE_BUFFER_SIZE];
uint8_t line_len = 0;

bool parse_bit(char*& save, uint8_t bit, uint8_t& flags) {
  char* tok = strtok_r(nullptr, " ", &save);
  if (!tok || (strcmp(tok, "0") != 0 && strcmp(tok, "1") != 0)) return false;
  if (tok[0] == '1') flags |= (uint8_t)(1u << bit);
  return true;
}

// `C <grams> <stable> <lid> <lock> <fault>` becomes one 0x180 frame. A
// garbled line is dropped: the host would take a wrong digit for a reading.
void forward(char* line) {
  char* save = nullptr;
  char* tok = strtok_r(line, " ", &save);
  if (!tok || strcmp(tok, "C") != 0) return;

  char* tok_grams = strtok_r(nullptr, " ", &save);
  if (!tok_grams) return;
  char* end = nullptr;
  const long grams = strtol(tok_grams, &end, 10);
  if (end == tok_grams || *end != 0 || grams < INT16_MIN || grams > INT16_MAX) return;

  uint8_t flags = 0;
  for (uint8_t bit = 0; bit < 4; bit++) {
    if (!parse_bit(save, bit, flags)) return;
  }
  if (strtok_r(nullptr, " ", &save)) return;

  struct can_frame frame;
  frame.can_id = config::CAN_CARGO_STATUS_ID;
  frame.can_dlc = 3;
  frame.data[0] = (uint8_t)((uint16_t)grams & 0xFF);
  frame.data[1] = (uint8_t)((uint16_t)grams >> 8);
  frame.data[2] = flags;
  bus->sendMessage(&frame);
}

void read_cargo() {
  while (Serial2.available()) {
    const int c = Serial2.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      line_buf[line_len] = 0;
      if (line_len > 0) forward(line_buf);
      line_len = 0;
    } else if (line_len < LINE_BUFFER_SIZE - 1) {
      line_buf[line_len++] = (char)c;
    } else {
      line_len = 0;
    }
  }
}

void read_bus() {
  struct can_frame frame;
  while (bus->readMessage(&frame) == MCP2515::ERROR_OK) {
    if (frame.can_id != config::CAN_CARGO_COMMAND_ID || frame.can_dlc < 1) continue;
    Serial2.print((frame.data[0] & 1) ? "K 1\n" : "K 0\n");
  }
}

}  // namespace

void init(MCP2515& mcp) {
  bus = &mcp;
  Serial2.begin(config::CARGO_BAUD, SERIAL_8N1, config::PIN_CARGO_RX, config::PIN_CARGO_TX);

  // Only the cargo command gets in: the lights frames, sent at the host's
  // rate, would otherwise fill both receive buffers during a ping cycle.
  mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  const MCP2515::RXF filters[] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2,
                                  MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
  for (const MCP2515::RXF filter : filters) {
    mcp.setFilter(filter, false, config::CAN_CARGO_COMMAND_ID);
  }
}

void update() {
  if (bus == nullptr) return;
  read_cargo();
  read_bus();
}

}  // namespace cargo_bridge
