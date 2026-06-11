#pragma once

#include <stdint.h>

namespace can_bus {

bool init();

bool send_lights(uint8_t bitmask);

}  // namespace can_bus
