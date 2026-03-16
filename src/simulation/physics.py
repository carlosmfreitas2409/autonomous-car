"""Differential drive physics simulation."""

import math
import time

from src.config import (
    FRICTION,
    MAX_RPM,
    MOTOR_TIME_CONSTANT,
    WHEEL_BASE,
    WHEEL_RADIUS,
)
from src.models.state import VehicleState


class DifferentialDrivePhysics:
    """Simulates a differential-drive robot using Euler integration."""

    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self.heading: float = 0.0
        self.v_left: float = 0.0
        self.v_right: float = 0.0

    def update(self, left_power: int, right_power: int, dt: float) -> None:
        """Advance the physics simulation by one time step.

        Args:
            left_power: Left motor PWM value (-255 to 255).
            right_power: Right motor PWM value (-255 to 255).
            dt: Time step in seconds.
        """
        if dt <= 0:
            return

        # 1. Convert motor power to target wheel velocity
        max_wheel_vel = MAX_RPM * 2.0 * math.pi * WHEEL_RADIUS / 60.0
        v_target_left = (left_power / 255.0) * max_wheel_vel
        v_target_right = (right_power / 255.0) * max_wheel_vel

        # 2. Apply first-order motor response (simulates inertia)
        alpha = min(1.0, dt / MOTOR_TIME_CONSTANT)
        self.v_left += (v_target_left - self.v_left) * alpha
        self.v_right += (v_target_right - self.v_right) * alpha

        # 3. Compute linear and angular velocity
        v_linear = (self.v_left + self.v_right) / 2.0
        omega = (self.v_right - self.v_left) / WHEEL_BASE

        # 4. Euler integration
        self.heading += omega * dt
        self.x += v_linear * math.cos(self.heading) * dt
        self.y += v_linear * math.sin(self.heading) * dt

        # 5. Apply friction
        self.v_left *= 1.0 - FRICTION * dt
        self.v_right *= 1.0 - FRICTION * dt

    def get_state(self) -> VehicleState:
        """Return the current vehicle state."""
        speed = (self.v_left + self.v_right) / 2.0
        return VehicleState(
            x=self.x,
            y=self.y,
            heading=self.heading,
            speed=abs(speed),
            left_motor=0,
            right_motor=0,
            timestamp=time.time(),
        )

    def reset(self) -> None:
        """Reset all state to initial values."""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.v_left = 0.0
        self.v_right = 0.0
