"""Command data models for vehicle control."""

from dataclasses import dataclass, field
import time


@dataclass
class ControlCommand:
    """High-level control command from input devices.

    Attributes:
        throttle: Forward/reverse power, -1.0 (full reverse) to 1.0 (full forward).
        steering: Left/right steering, -1.0 (full left) to 1.0 (full right).
        emergency_stop: If True, immediately stop all motors.
        timestamp: Time the command was created (seconds since epoch).
    """
    throttle: float = 0.0    # -1..1
    steering: float = 0.0    # -1..1
    emergency_stop: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class MotorCommand:
    """Low-level motor command sent to the vehicle hardware.

    Attributes:
        left: Left motor PWM value, -255 to 255.
        right: Right motor PWM value, -255 to 255.
        timestamp: Time the command was created (seconds since epoch).
    """
    left: int = 0      # -255..255
    right: int = 0     # -255..255
    timestamp: float = field(default_factory=time.time)
