import pygame


_STICK_AXIS = 0  # Left stick X


class PS5Controller:
    def __init__(self, deadzone: float = 0.08):
        self._deadzone = deadzone
        self._joystick: pygame.joystick.JoystickType | None = None

        pygame.joystick.init()
        self._try_connect()

    def _try_connect(self) -> None:
        if pygame.joystick.get_count() > 0:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
        else:
            self._joystick = None

    def poll(self) -> float:
        """Return steering in [-1.0, 1.0] from the left stick X-axis."""
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
                return 0.0

        raw = self._joystick.get_axis(_STICK_AXIS)

        if abs(raw) < self._deadzone:
            return 0.0

        sign = 1.0 if raw > 0 else -1.0
        scaled = (abs(raw) - self._deadzone) / (1.0 - self._deadzone)
        return sign * min(scaled, 1.0)

    def is_connected(self) -> bool:
        return self._joystick is not None

    def shutdown(self) -> None:
        pygame.joystick.quit()
