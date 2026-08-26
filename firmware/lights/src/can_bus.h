#pragma once

#include <stdint.h>

namespace can_bus {

bool init();

bool poll(uint8_t& bitmask, uint8_t& effect);

}
