"""PS5 DualSense controller input."""

import math

import pygame

from src.config import DEADZONE_THRESHOLD


def _apply_deadzone(value: float, threshold: float) -> float:
    """Snap values below threshold to 0, rescale the rest to full range."""
    if abs(value) < threshold:
        return 0.0
    sign = math.copysign(1.0, value)
    return sign * (abs(value) - threshold) / (1.0 - threshold)


class PS5Controller:
    """Reads input from a PS5 DualSense controller.

    Mapping:
        R2 trigger   (axis 5) — throttle (0..1, forward only)
        Left stick X (axis 0) — steering
        Circle       (btn 1)  — toggle emergency stop
    """

    def __init__(self):
        self._joystick: pygame.joystick.JoystickType | None = None
        self._emergency_stop = False
        self._prev_circle = False

        try:
            if pygame.joystick.get_count() > 0:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
        except pygame.error:
            self._joystick = None

    def read(self) -> tuple[float, float, bool]:
        """Read controller input. Returns (throttle, steering, emergency_stop)."""
        if not self.is_connected():
            return (0.0, 0.0, True)

        try:
            r2_raw = self._joystick.get_axis(5)
            raw_throttle = (r2_raw + 1.0) / 2.0

            raw_steering = -self._joystick.get_axis(0)

            throttle = _apply_deadzone(raw_throttle, DEADZONE_THRESHOLD)
            steering = _apply_deadzone(raw_steering, DEADZONE_THRESHOLD)

            circle_pressed = bool(self._joystick.get_button(1))
            if circle_pressed and not self._prev_circle:
                self._emergency_stop = not self._emergency_stop
            self._prev_circle = circle_pressed

            return (throttle, steering, self._emergency_stop)
        except pygame.error:
            return (0.0, 0.0, True)

    def is_connected(self) -> bool:
        if self._joystick is None:
            return False
        try:
            return self._joystick.get_init()
        except pygame.error:
            return False
