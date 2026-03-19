"""Tunable constants for the RC car."""

# Serial (Arduino Mega typically appears as /dev/cu.usbmodemXXXX on macOS)
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
COMMAND_FREQUENCY = 30         # commands per second sent to Arduino

# Control
DEADZONE_THRESHOLD = 0.08      # input magnitude below this is treated as zero
THROTTLE_RAMP_RATE = 3.0       # max throttle change per second
STEERING_RAMP_RATE = 5.0       # max steering change per second

# Safety
WATCHDOG_TIMEOUT = 0.5         # seconds before watchdog triggers emergency stop

# Loop
TICK_RATE = 60                 # main loop frequency (Hz)
