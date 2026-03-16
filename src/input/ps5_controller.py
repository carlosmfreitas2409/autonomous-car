"""PS5 DualSense controller input via pygame joystick."""

import pygame

from src.config import DEADZONE_THRESHOLD
from src.input.base import InputController
from src.models.commands import ControlCommand


def _apply_deadzone(value: float, threshold: float) -> float:
    """Return 0.0 if the absolute value is below the threshold, else the original value."""
    if abs(value) < threshold:
        return 0.0
    return value


class PS5Controller(InputController):
    """Reads input from a PS5 DualSense controller.

    Mapping:
        Left stick Y-axis  (axis 1) — throttle (inverted: up = positive)
        Right stick X-axis  (axis 2) — steering
        Circle button       (btn 1)  — toggle emergency stop
        Triangle button     (btn 3)  — reset emergency stop
    """

    def __init__(self):
        self._joystick: pygame.joystick.JoystickType | None = None
        self._emergency_stop = False
        self._prev_circle = False
        self._prev_triangle = False

        try:
            if pygame.joystick.get_count() > 0:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
        except pygame.error:
            self._joystick = None

    def read(self) -> ControlCommand:
        if not self.is_connected():
            return ControlCommand(throttle=0.0, steering=0.0, emergency_stop=True)

        try:
            # Axes
            raw_throttle = -self._joystick.get_axis(1)  # invert: up is negative in pygame
            raw_steering = self._joystick.get_axis(2)

            throttle = _apply_deadzone(raw_throttle, DEADZONE_THRESHOLD)
            steering = _apply_deadzone(raw_steering, DEADZONE_THRESHOLD)

            # Circle button (button 1) — toggle emergency stop (edge-triggered)
            circle_pressed = bool(self._joystick.get_button(1))
            if circle_pressed and not self._prev_circle:
                self._emergency_stop = True
            self._prev_circle = circle_pressed

            # Triangle button (button 3) — reset emergency stop (edge-triggered)
            triangle_pressed = bool(self._joystick.get_button(3))
            if triangle_pressed and not self._prev_triangle:
                self._emergency_stop = False
            self._prev_triangle = triangle_pressed

            return ControlCommand(
                throttle=throttle,
                steering=steering,
                emergency_stop=self._emergency_stop,
            )
        except pygame.error:
            return ControlCommand(throttle=0.0, steering=0.0, emergency_stop=True)

    def is_connected(self) -> bool:
        if self._joystick is None:
            return False
        try:
            return self._joystick.get_init()
        except pygame.error:
            return False
