#pragma once

#include <stdint.h>

namespace ultrasonic {

void init();

uint8_t read_cm(uint8_t index);

}
