#pragma once

#include <stdint.h>

namespace cargo {

void on_frame(const uint8_t* data, uint8_t dlc);

void update();

}
