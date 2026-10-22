#pragma once

#include <stdint.h>

namespace scale {

// Tares with the bed empty: power the node up with the compartment empty.
bool init();
void update();

int16_t grams();
bool stable();
bool fault();

}  // namespace scale
