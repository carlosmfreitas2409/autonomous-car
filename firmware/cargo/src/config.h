#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace config {

constexpr uint32_t SERIAL_BAUD = 115200;

// UART2 to the sonar node (its RX2 16 / TX2 17), which bridges onto CAN.
// Both are 3.3 V boards: wired straight, with a common ground.
constexpr uint8_t PIN_LINK_TX = 32;
constexpr uint8_t PIN_LINK_RX = 33;
constexpr uint32_t LINK_BAUD = 115200;

constexpr uint8_t PIN_SCALE_DT = 26;
constexpr uint8_t PIN_SCALE_SCK = 25;

// Limit switch to 3V3 with a 10k pull-down: HIGH = the lid presses it shut. A
// broken wire reads open, so the truck will not leave.
constexpr uint8_t PIN_LID = 13;

// The lock's driver input, as on the bench sketch: LOW powers the lock (bolt
// pulled in, unlocked), HIGH leaves it to the spring (locked). GPIO 2 also
// drives the board's LED.
constexpr uint8_t PIN_LOCK = 2;
constexpr uint8_t LOCK_ON = LOW;
constexpr uint8_t LOCK_OFF = HIGH;

constexpr uint32_t STATUS_PERIOD_MS = 100;

// Measured with tools/cargo_test: counts per gram for this load cell.
constexpr float SCALE_COUNTS_PER_GRAM = 423.4243f;
constexpr uint8_t SCALE_TARE_SAMPLES = 20;
constexpr uint32_t SCALE_READY_TIMEOUT_MS = 1000;
constexpr uint32_t SCALE_FAULT_MS = 500;       // no reading this long: the HX711 is not answering
constexpr uint8_t SCALE_AVERAGE = 5;           // reported grams: mean of the last 5 readings (0.5 s)
constexpr uint8_t SCALE_STABLE_WINDOW = 10;    // stable: the last 10 readings (1 s) ...
constexpr float SCALE_STABLE_SPAN_G = 5.0f;    // ... within 5 g of each other

constexpr uint32_t LID_DEBOUNCE_MS = 50;

// The host's unlock is a lease: it must be renewed this often or the lock
// drops, so a dead host, master or bus leaves the compartment locked.
constexpr uint32_t UNLOCK_LEASE_MS = 300;
// The coil is not rated to be powered for long: after this it rests,
// whatever the host asks.
constexpr uint32_t LOCK_MAX_ON_MS = 60000;
constexpr uint32_t LOCK_REST_MS = 20000;

constexpr uint32_t DEBUG_PERIOD_MS = 500;

}  // namespace config
