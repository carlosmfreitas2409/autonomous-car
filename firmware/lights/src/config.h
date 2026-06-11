#pragma once

#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint8_t PIN_RELAY_RIGHT_TURN = 8;
constexpr uint8_t PIN_RELAY_LEFT_TURN  = 7;
constexpr uint8_t PIN_RELAY_DRL        = 14;
constexpr uint8_t PIN_RELAY_LOW_BEAM   = 9;
constexpr uint8_t PIN_RELAY_BRAKE      = 3;
constexpr uint8_t PIN_RELAY_REVERSE    = 4;
constexpr uint8_t PIN_RELAY_LOGO       = 6;
constexpr uint8_t PIN_RELAY_LED        = 5;


constexpr uint8_t PIN_STATUS_LED = 13;

constexpr uint8_t PIN_CAN_CS  = 10;
constexpr uint8_t PIN_CAN_INT = 2;

constexpr uint16_t CAN_LIGHTS_ID = 0x100;

constexpr uint32_t CAN_WATCHDOG_MS = 1000;

constexpr uint16_t BLINK_ON_MS  = 400;
constexpr uint16_t BLINK_OFF_MS = 250;

}
