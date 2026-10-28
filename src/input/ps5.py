from typing import Optional

import pygame

from path import HOME_PATH, Path


_STICK_AXIS = 0
_RTRIGGER_AXIS = 5
_LTRIGGER_AXIS = 4
_YBUTTON_AXIS = 3
_BBUTTON = 1
_CROSS_BUTTON = 0
_L1_BUTTON = 9
_R1_BUTTON = 10
_TRIGGER_DEADZONE = 0.05

BUTTON_NAMES = {
    0: "Cross", 1: "Circle", 2: "Square", 3: "Triangle",
    4: "Share", 5: "PS", 6: "Options", 7: "L3", 8: "R3",
    9: "L1", 10: "R1", 11: "D-up", 12: "D-down", 13: "D-left", 14: "D-right",
    15: "Touchpad",
}

class PS5Controller:
    def __init__(self, deadzone: float = 0.08):
        self._deadzone = deadzone
        self._joystick: pygame.joystick.JoystickType | None = None
        self._y_was_pressed = False
        self._b_was_pressed = False
        self._cross_was_pressed = False
        self._l1_was_pressed = False
        self._r1_was_pressed = False
        self._buttons_were_pressed: set[int] = set()

        if not pygame.display.get_init():
            pygame.display.init()
        pygame.joystick.init()
        self._try_connect()

    def _try_connect(self) -> None:
        if pygame.joystick.get_count() > 0:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
        else:
            self._joystick = None

    def _update(self) -> bool:
        pygame.event.pump()

        for event in pygame.event.get(
            [pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED]
        ):
            if event.type == pygame.JOYDEVICEREMOVED:
                self._joystick = None
            elif event.type == pygame.JOYDEVICEADDED:
                self._try_connect()

        if self._joystick is None:
            self._try_connect()
        return self._joystick is not None

    def poll_estop_toggle(self) -> bool:
        if not self._update():
            self._y_was_pressed = False
            return False

        pressed = self._joystick.get_button(_YBUTTON_AXIS)
        toggled = pressed and not self._y_was_pressed
        self._y_was_pressed = pressed
        return toggled

    def poll_mode_toggle(self) -> bool:
        if not self._update():
            self._b_was_pressed = False
            return False

        pressed = self._joystick.get_button(_BBUTTON)
        toggled = pressed and not self._b_was_pressed
        self._b_was_pressed = pressed
        return toggled

    def poll_release(self) -> bool:
        if not self._update():
            self._cross_was_pressed = False
            return False

        pressed = self._joystick.get_button(_CROSS_BUTTON)
        released = pressed and not self._cross_was_pressed
        self._cross_was_pressed = pressed
        return released

    def poll_path_buttons(self) -> Optional[Path]:
        if not self._update():
            self._l1_was_pressed = False
            self._r1_was_pressed = False
            return None

        l1 = self._joystick.get_button(_L1_BUTTON)
        r1 = self._joystick.get_button(_R1_BUTTON)
        l1_edge = l1 and not self._l1_was_pressed
        r1_edge = r1 and not self._r1_was_pressed
        self._l1_was_pressed = l1
        self._r1_was_pressed = r1
        if l1_edge:
            return HOME_PATH
        if r1_edge:
            return Path.DETOUR
        return None

    def poll_button_presses(self) -> list[int]:
        if not self._update():
            self._buttons_were_pressed = set()
            return []
        pressed = {i for i in range(self._joystick.get_numbuttons()) if self._joystick.get_button(i)}
        edges = sorted(pressed - self._buttons_were_pressed)
        self._buttons_were_pressed = pressed
        return edges

    def poll(self) -> tuple[float, float]:
        if not self._update():
            return 0.0, 0.0

        raw_stick = self._joystick.get_axis(_STICK_AXIS)
        if abs(raw_stick) < self._deadzone:
            steering = 0.0
        else:
            sign = 1.0 if raw_stick > 0 else -1.0
            scaled = (abs(raw_stick) - self._deadzone) / (1.0 - self._deadzone)
            steering = sign * min(scaled, 1.0)

        raw_rtrigger = self._joystick.get_axis(_RTRIGGER_AXIS)
        rtrigger = (raw_rtrigger + 1.0) / 2.0
        if rtrigger < _TRIGGER_DEADZONE:
            rtrigger = 0.0

        raw_ltrigger = self._joystick.get_axis(_LTRIGGER_AXIS)
        ltrigger = (raw_ltrigger + 1.0) / 2.0
        if ltrigger < _TRIGGER_DEADZONE:
            ltrigger = 0.0

        throttle = rtrigger - ltrigger
        return steering, throttle

    def is_connected(self) -> bool:
        return self._joystick is not None

    def shutdown(self) -> None:
        pygame.joystick.quit()
        if pygame.display.get_init():
            pygame.display.quit()
