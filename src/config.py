"""Tunable constants for the autonomous vehicle system."""

# --- Mode ---
MODE = "sim"                   # "sim" or "arduino"

# --- Physics ---
WHEEL_BASE = 0.15              # meters, distance between wheels
WHEEL_RADIUS = 0.033           # meters
MAX_RPM = 200                  # maximum motor RPM
FRICTION = 0.5                 # friction coefficient
MOTOR_TIME_CONSTANT = 0.1      # seconds, motor response time constant

# --- Control ---
DEADZONE_THRESHOLD = 0.08      # input magnitude below this is treated as zero
THROTTLE_RAMP_RATE = 3.0       # max throttle change per second
STEERING_RAMP_RATE = 5.0       # max steering change per second

# --- Simulation ---
TICK_RATE = 60                 # frames per second
SCREEN_WIDTH = 800             # pixels
SCREEN_HEIGHT = 600            # pixels
PIXELS_PER_METER = 200         # display scale

# --- Safety ---
WATCHDOG_TIMEOUT = 0.5         # seconds before watchdog triggers emergency stop

# --- Serial ---
SERIAL_PORT = "/dev/ttyUSB0"   # default serial port for Arduino
SERIAL_BAUD = 115200           # baud rate
COMMAND_FREQUENCY = 30         # commands per second sent to Arduino

# --- Logging ---
LOG_ENABLED = False            # set True to write sim_log.csv
LOG_FILE = "sim_log.csv"       # output CSV filename
