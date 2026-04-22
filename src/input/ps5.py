import pygame


_STICK_AXIS = 0  # Left stick X
_TRIGGER_AXIS = 5  # R2 (DualSense via SDL2 on macOS)
_TRIGGER_DEADZONE = 0.05


class PS5Controller:
    def __init__(self, deadzone: float = 0.08):
        self._deadzone = deadzone
        self._joystick: pygame.joystick.JoystickType | None = None

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

    def poll(self) -> tuple[float, float]:
        """Return (steering, throttle). Steering in [-1, 1] from left stick X; throttle in [0, 1] from R2."""
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
            if self._joystick is None:
                return 0.0, 0.0

        raw_stick = self._joystick.get_axis(_STICK_AXIS)
        if abs(raw_stick) < self._deadzone:
            steering = 0.0
        else:
            sign = 1.0 if raw_stick > 0 else -1.0
            scaled = (abs(raw_stick) - self._deadzone) / (1.0 - self._deadzone)
            steering = sign * min(scaled, 1.0)

        raw_trigger = self._joystick.get_axis(_TRIGGER_AXIS)
        throttle = (raw_trigger + 1.0) / 2.0
        if throttle < _TRIGGER_DEADZONE:
            throttle = 0.0

        return steering, throttle

    def is_connected(self) -> bool:
        return self._joystick is not None

    def shutdown(self) -> None:
        pygame.joystick.quit()
        if pygame.display.get_init():
            pygame.display.quit()
