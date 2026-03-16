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
        R2 trigger   (axis 5) — throttle (0..1, forward only)
        Left stick X (axis 0) — steering
        Circle       (btn 1)  — emergency stop (held = stopped, released = resume)
    """

    def __init__(self):
        self._joystick: pygame.joystick.JoystickType | None = None

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
            # R2 (axis 5): range -1 (rest) to 1 (fully pressed) → 0..1
            r2_raw = self._joystick.get_axis(5)
            raw_throttle = (r2_raw + 1.0) / 2.0

            # Left stick X (axis 0) for steering
            raw_steering = self._joystick.get_axis(0)

            throttle = _apply_deadzone(raw_throttle, DEADZONE_THRESHOLD)
            steering = _apply_deadzone(raw_steering, DEADZONE_THRESHOLD)

            # Circle button (button 1) — hold to stop
            emergency_stop = bool(self._joystick.get_button(1))

            return ControlCommand(
                throttle=throttle,
                steering=steering,
                emergency_stop=emergency_stop,
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
