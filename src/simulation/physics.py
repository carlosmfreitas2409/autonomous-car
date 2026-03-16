"""Bicycle model physics simulation for a 4-wheeled RC car."""

import math
import time

from src.config import (
    FRICTION,
    MAX_RPM,
    MAX_STEERING_ANGLE,
    MOTOR_TIME_CONSTANT,
    WHEEL_BASE,
    WHEEL_RADIUS,
)
from src.models.state import VehicleState


class BicyclePhysics:
    """Simulates a car using the kinematic bicycle model.

    The bicycle model treats the car as having one front wheel (steered)
    and one rear wheel (driven). This prevents unrealistic behaviors like
    spinning in place that differential drive allows.

    The turning radius is determined by wheelbase and steering angle:
        radius = wheelbase / tan(steering_angle)
    """

    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self.heading: float = 0.0
        self.speed: float = 0.0
        self._steering_angle: float = 0.0  # current front wheel angle (radians)

    def update(self, throttle: float, steering: float, dt: float) -> None:
        """Advance the physics simulation by one time step.

        Args:
            throttle: Throttle input 0.0 to 1.0 (forward only).
            steering: Steering input -1.0 to 1.0 (left/right).
            dt: Time step in seconds.
        """
        if dt <= 0:
            return

        # 1. Target speed from throttle
        max_speed = MAX_RPM * 2.0 * math.pi * WHEEL_RADIUS / 60.0
        target_speed = throttle * max_speed

        # 2. First-order response for speed (motor inertia)
        alpha = min(1.0, dt / MOTOR_TIME_CONSTANT)
        self.speed += (target_speed - self.speed) * alpha

        # 3. Target steering angle
        target_angle = steering * MAX_STEERING_ANGLE
        # Steering responds faster than throttle
        steering_alpha = min(1.0, dt / (MOTOR_TIME_CONSTANT * 0.5))
        self._steering_angle += (target_angle - self._steering_angle) * steering_alpha

        # 4. Bicycle model kinematics (rear axle reference)
        if abs(self._steering_angle) > 1e-6:
            turn_radius = WHEEL_BASE / math.tan(self._steering_angle)
            omega = self.speed / turn_radius
        else:
            omega = 0.0

        # 5. Euler integration
        self.heading += omega * dt
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

        # 6. Friction (deceleration when no throttle)
        self.speed *= 1.0 - FRICTION * dt
        if abs(self.speed) < 0.001:
            self.speed = 0.0

    def get_state(self) -> VehicleState:
        """Return the current vehicle state."""
        return VehicleState(
            x=self.x,
            y=self.y,
            heading=self.heading,
            speed=abs(self.speed),
            left_motor=0,
            right_motor=0,
            timestamp=time.time(),
        )

    def reset(self) -> None:
        """Reset all state to initial values."""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self._steering_angle = 0.0
