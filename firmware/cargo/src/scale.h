#pragma once

#include <stdint.h>

namespace scale {
bool init();
void update();

int16_t grams();
bool stable();
bool fault();

}
