"""Bicycle model physics for the simulator. No pygame dependency."""

import math
import time
from dataclasses import dataclass, field

# Sim-only physics constants
WHEEL_BASE = 0.25
WHEEL_RADIUS = 0.033
MAX_RPM = 200
MAX_STEERING_ANGLE = 0.45
FRICTION = 0.5
MOTOR_TIME_CONSTANT = 0.1


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    sensors: dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


class BicyclePhysics:
    """Kinematic bicycle model."""

    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self.heading: float = 0.0
        self.speed: float = 0.0
        self._steering_angle: float = 0.0

    def update(self, throttle: float, steering: float, dt: float) -> None:
        if dt <= 0:
            return

        max_speed = MAX_RPM * 2.0 * math.pi * WHEEL_RADIUS / 60.0
        target_speed = throttle * max_speed

        alpha = min(1.0, dt / MOTOR_TIME_CONSTANT)
        self.speed += (target_speed - self.speed) * alpha

        target_angle = steering * MAX_STEERING_ANGLE
        steering_alpha = min(1.0, dt / (MOTOR_TIME_CONSTANT * 0.5))
        self._steering_angle += (target_angle - self._steering_angle) * steering_alpha

        if abs(self._steering_angle) > 1e-6:
            turn_radius = WHEEL_BASE / math.tan(self._steering_angle)
            omega = self.speed / turn_radius
        else:
            omega = 0.0

        self.heading += omega * dt
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

        self.speed *= 1.0 - FRICTION * dt
        if abs(self.speed) < 0.001:
            self.speed = 0.0

    def get_state(self) -> VehicleState:
        return VehicleState(
            x=self.x, y=self.y, heading=self.heading,
            speed=abs(self.speed), timestamp=time.time(),
        )

    def reset(self) -> None:
        self.x = self.y = self.heading = self.speed = 0.0
        self._steering_angle = 0.0
