#pragma once

#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint8_t PIN_RELAY_RIGHT_TURN = 2;
constexpr uint8_t PIN_RELAY_LEFT_TURN  = 3;
constexpr uint8_t PIN_RELAY_DRL        = 4;
constexpr uint8_t PIN_RELAY_LOW_BEAM   = 5;
constexpr uint8_t PIN_RELAY_BRAKE      = 6;
constexpr uint8_t PIN_RELAY_REVERSE    = 7;
constexpr uint8_t PIN_RELAY_LOGO       = 8;

constexpr uint8_t PIN_STATUS_LED = 13;

// Turn-signal cadence. ON must be long enough for the front sequential module
// to finish a full sweep — bench-time it with a stopwatch and bump if a sweep
// gets cut off mid-animation. OFF is the visible blink gap between cycles.
constexpr uint16_t BLINK_ON_MS  = 400;
constexpr uint16_t BLINK_OFF_MS = 250;

}
