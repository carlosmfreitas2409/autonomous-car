#pragma once

#include <stdint.h>

namespace can_bus {

bool init();

bool send_lights(uint8_t bitmask, uint8_t effect);

}  // namespace can_bus
