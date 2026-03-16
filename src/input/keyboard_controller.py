"""Keyboard-based input controller using pygame."""

import pygame

from src.input.base import InputController
from src.models.commands import ControlCommand


class KeyboardController(InputController):
    """Reads input from keyboard keys via pygame.

    Controls:
        W / Up Arrow    — throttle +1
        S / Down Arrow  — throttle -1
        A / Left Arrow  — steering -1
        D / Right Arrow — steering +1
        Space           — toggle emergency stop on
        R               — reset (resume from) emergency stop
    """

    def __init__(self):
        self._emergency_stop = False
        self._prev_space = False
        self._prev_r = False

    def read(self) -> ControlCommand:
        keys = pygame.key.get_pressed()

        # Throttle
        throttle = 0.0
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            throttle += 1.0
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            throttle -= 1.0

        # Steering
        steering = 0.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            steering -= 1.0
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            steering += 1.0

        # Emergency stop toggle (edge-triggered)
        space_pressed = bool(keys[pygame.K_SPACE])
        if space_pressed and not self._prev_space:
            self._emergency_stop = not self._emergency_stop
        self._prev_space = space_pressed

        # Reset emergency stop (edge-triggered)
        r_pressed = bool(keys[pygame.K_r])
        if r_pressed and not self._prev_r:
            self._emergency_stop = False
        self._prev_r = r_pressed

        return ControlCommand(
            throttle=throttle,
            steering=steering,
            emergency_stop=self._emergency_stop,
        )

    def is_connected(self) -> bool:
        return True
