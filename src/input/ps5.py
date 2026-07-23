import pygame


_STICK_AXIS = 0  # Left stick X
_RTRIGGER_AXIS = 5  # R2 (DualSense via SDL2 on macOS)
_LTRIGGER_AXIS = 4
_YBUTTON_AXIS = 3
_BBUTTON = 1  # Circle (DualSense via SDL2 on macOS)
_TRIGGER_DEADZONE = 0.05

class PS5Controller:
    def __init__(self, deadzone: float = 0.08):
        self._deadzone = deadzone
        self._joystick: pygame.joystick.JoystickType | None = None
        self._y_was_pressed = False
        self._b_was_pressed = False

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
