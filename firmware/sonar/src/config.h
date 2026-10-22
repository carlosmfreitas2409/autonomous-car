#pragma once

#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint8_t SENSOR_COUNT = 3;
constexpr uint8_t PIN_TRIG[SENSOR_COUNT] = {27, 26, 25};
constexpr uint8_t PIN_ECHO[SENSOR_COUNT] = {35, 32, 33};

constexpr uint8_t PIN_STATUS_LED = 2;
constexpr uint8_t PIN_CAN_CS = 5;

constexpr uint16_t CAN_SONAR_ID = 0x080;
constexpr uint32_t CAN_PERIOD_MS = 50;

constexpr uint16_t MAX_RANGE_CM = 200;

constexpr uint32_t ECHO_TIMEOUT_US = (uint32_t)MAX_RANGE_CM * 58;

constexpr uint32_t PING_SETTLE_MS = 5;

constexpr uint8_t DIST_NONE = 0xFF;  // nothing in range

// The cargo node (firmware/cargo) on UART2: its `C` lines go out as CAN
// 0x180, the master's 0x181 comes back to it as `K 0|1`.
constexpr uint8_t PIN_CARGO_RX = 16;   // from the cargo node's GPIO 32
constexpr uint8_t PIN_CARGO_TX = 17;   // to the cargo node's GPIO 33
constexpr uint32_t CARGO_BAUD = 115200;
constexpr uint16_t CAN_CARGO_STATUS_ID = 0x180;
constexpr uint16_t CAN_CARGO_COMMAND_ID = 0x181;

}  // namespace config
