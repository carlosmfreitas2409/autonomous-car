#pragma once

#include <stdint.h>

namespace can_bus {

bool init();

bool send_lights(uint8_t bitmask, uint8_t effect);

bool send_cargo(uint8_t flags);

// A cargo status frame read here is handed to cargo::on_frame.
bool poll_sonar(uint8_t data[8]);

}  // namespace can_bus
