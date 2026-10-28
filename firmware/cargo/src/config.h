#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint8_t PIN_LINK_TX = 32;
constexpr uint8_t PIN_LINK_RX = 33;
constexpr uint32_t LINK_BAUD = 115200;

constexpr uint8_t PIN_SCALE_DT = 26;
constexpr uint8_t PIN_SCALE_SCK = 25;

constexpr uint8_t PIN_LID = 13;

constexpr uint8_t PIN_LOCK = 2;
constexpr uint8_t LOCK_ON = LOW;
constexpr uint8_t LOCK_OFF = HIGH;

constexpr uint32_t STATUS_PERIOD_MS = 100;

constexpr float SCALE_COUNTS_PER_GRAM = 423.4243f;
constexpr uint8_t SCALE_TARE_SAMPLES = 20;
constexpr uint32_t SCALE_READY_TIMEOUT_MS = 1000;
constexpr uint32_t SCALE_FAULT_MS = 500;
constexpr uint8_t SCALE_AVERAGE = 5;
constexpr uint8_t SCALE_STABLE_WINDOW = 10;
constexpr float SCALE_STABLE_SPAN_G = 5.0f;

constexpr uint32_t LID_DEBOUNCE_MS = 50;

constexpr uint32_t UNLOCK_LEASE_MS = 300;
constexpr uint32_t LOCK_MAX_ON_MS = 60000;
constexpr uint32_t LOCK_REST_MS = 20000;

constexpr uint32_t DEBUG_PERIOD_MS = 500;

}
