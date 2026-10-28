#pragma once

namespace battery {

void init();

void update();

// The smoothed readings behind the LED bar: battery volts and charge (0-100).
// Under motor load the voltage sags, so both read low while driving.
float volts();
float percent();

}  // namespace battery
