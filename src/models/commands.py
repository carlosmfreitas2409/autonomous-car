"""Command data models for vehicle control."""

import time
from dataclasses import dataclass, field


@dataclass
class ControlCommand:
    """High-level control command from input devices.

    Attributes:
        throttle: Forward power, 0.0 (stopped) to 1.0 (full forward).
        steering: Left/right steering, -1.0 (full left) to 1.0 (full right).
        emergency_stop: If True, immediately stop all motors.
        timestamp: Time the command was created (seconds since epoch).
    """
    throttle: float = 0.0
    steering: float = 0.0
    emergency_stop: bool = False
    timestamp: float = field(default_factory=time.time)

    def __post_init__(self):
        self.throttle = max(0.0, min(1.0, self.throttle))
        self.steering = max(-1.0, min(1.0, self.steering))


@dataclass
class MotorCommand:
    """Low-level motor command sent to the vehicle hardware.

    Attributes:
        left: Left motor PWM value, -255 to 255.
        right: Right motor PWM value, -255 to 255.
        timestamp: Time the command was created (seconds since epoch).
    """
    left: int = 0
    right: int = 0
    timestamp: float = field(default_factory=time.time)

    def __post_init__(self):
        self.left = max(-255, min(255, int(self.left)))
        self.right = max(-255, min(255, int(self.right)))
