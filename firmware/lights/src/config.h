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
constexpr uint8_t PIN_RELAY_PROJECTOR  = 5;

constexpr uint8_t  PIN_STRIP_DATA        = 16;
constexpr uint16_t STRIP_LED_COUNT       = 22;
constexpr uint16_t STRIP_FRONT_COUNT     = 9;
constexpr uint16_t STRIP_REAR_COUNT      = 3;
constexpr uint8_t  STRIP_BRIGHTNESS      = 200;
constexpr uint8_t  STRIP_FRAME_MS        = 10;
constexpr uint16_t STRIP_REFRESH_MS      = 250;
constexpr uint16_t STROBE_ON_MS          = 40;
constexpr uint16_t STROBE_OFF_MS         = 60;
constexpr uint16_t PULSE_STEP            = 437;
constexpr uint16_t RAINBOW_HUE_STEP      = 256;
constexpr uint16_t POLICE_FLASH_MS       = 50;
constexpr uint16_t POLICE_GAP_MS         = 50;
constexpr uint16_t POLICE_PAUSE_MS       = 180;

constexpr uint8_t PIN_STATUS_LED = 13;

constexpr uint8_t PIN_CAN_CS  = 10;
constexpr uint8_t PIN_CAN_INT = 2;

constexpr uint16_t CAN_LIGHTS_ID = 0x100;

constexpr uint32_t CAN_WATCHDOG_MS = 1000;

constexpr uint16_t BLINK_ON_MS  = 400;
constexpr uint16_t BLINK_OFF_MS = 250;

}
