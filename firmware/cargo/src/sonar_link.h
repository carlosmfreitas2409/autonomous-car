#pragma once

#include <stdint.h>

// The serial link to the sonar node, which bridges it onto CAN to the master:
// `C` status lines out, `K` commands in.
namespace sonar_link {

void init();

// Reads the commands the sonar node passes on; call every loop.
void update();

void send_status(int16_t grams, bool stable, bool lid_closed, bool unlocked, bool fault);

// The host renewed the unlock within the lease.
bool unlock_requested();

}  // namespace sonar_link
