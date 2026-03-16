"""Vehicle state data model."""

from dataclasses import dataclass, field
import time


@dataclass
class VehicleState:
    """Complete state of the vehicle at a point in time.

    Attributes:
        x: X position in meters.
        y: Y position in meters.
        heading: Heading angle in radians (0 = east, pi/2 = north).
        speed: Current speed in meters per second.
        left_motor: Current left motor PWM value (-255 to 255).
        right_motor: Current right motor PWM value (-255 to 255).
        sensors: Dictionary of sensor readings.
        timestamp: Time this state was recorded (seconds since epoch).
    """
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    left_motor: int = 0
    right_motor: int = 0
    sensors: dict = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
