#pragma once

#include <stdint.h>

namespace sonar_link {

void init();

void update();

void send_status(int16_t grams, bool stable, bool lid_closed, bool unlocked, bool fault);

bool unlock_requested();

}
