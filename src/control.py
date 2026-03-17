"""Control pipeline: smoothing, e-stop, and watchdog."""

import time
from enum import Enum, auto

from src.config import THROTTLE_RAMP_RATE, STEERING_RAMP_RATE, WATCHDOG_TIMEOUT


class Smoother:
    """Limits the rate of change of a value over time."""

    def __init__(self, max_rate_per_second: float):
        self._max_rate = max_rate_per_second
        self._current: float = 0.0

    def smooth(self, target: float, dt: float) -> float:
        max_change = self._max_rate * dt
        diff = target - self._current
        if abs(diff) <= max_change:
            self._current = target
        else:
            self._current += max_change if diff > 0 else -max_change
        return self._current

    def reset(self):
        self._current = 0.0


class _EStopState(Enum):
    RUNNING = auto()
    STOPPED = auto()


class EStopManager:
    """Emergency stop state machine."""

    def __init__(self):
        self._state = _EStopState.RUNNING

    def activate(self) -> None:
        self._state = _EStopState.STOPPED

    def reset(self) -> None:
        self._state = _EStopState.RUNNING

    def is_active(self) -> bool:
        return self._state is _EStopState.STOPPED


class CommandWatchdog:
    """Triggers safety action if no command arrives within timeout."""

    def __init__(self, timeout: float = WATCHDOG_TIMEOUT):
        self._timeout = timeout
        self._last_feed: float = time.monotonic()

    def feed(self) -> None:
        self._last_feed = time.monotonic()

    def is_expired(self) -> bool:
        return (time.monotonic() - self._last_feed) > self._timeout


class Controller:
    """Smooths throttle/steering and manages e-stop."""

    def __init__(self, estop: EStopManager):
        self._throttle_smoother = Smoother(THROTTLE_RAMP_RATE)
        self._steering_smoother = Smoother(STEERING_RAMP_RATE)
        self._estop = estop

    def process(
        self, throttle: float, steering: float, emergency_stop: bool, dt: float
    ) -> tuple[float, float]:
        """Process input through smoothing and e-stop. Returns (throttle, steering)."""
        if emergency_stop:
            self._estop.activate()

        if self._estop.is_active():
            self._throttle_smoother.reset()
            self._steering_smoother.reset()
            return (0.0, 0.0)

        throttle = self._throttle_smoother.smooth(throttle, dt)
        steering = self._steering_smoother.smooth(steering, dt)
        return (throttle, steering)
