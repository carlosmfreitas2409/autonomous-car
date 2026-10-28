#pragma once

namespace lock {

void init();

void update(bool requested, bool lid_closed);

bool unlocked();

}
