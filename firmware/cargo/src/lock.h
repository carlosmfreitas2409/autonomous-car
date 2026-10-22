#pragma once

namespace lock {

void init();

// Powers the coil (unlocks) only while `requested` and the lid is closed: once
// the lid is lifted the bolt springs out, and the slanted latch catches again
// as the lid shuts.
void update(bool requested, bool lid_closed);

bool unlocked();

}  // namespace lock
