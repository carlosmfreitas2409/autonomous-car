"""Acceleration ramp smoother for control inputs."""


class Smoother:
    """Limits the rate of change of a value over time.

    Args:
        max_rate_per_second: Maximum allowed change in value per second.
    """

    def __init__(self, max_rate_per_second: float):
        self._max_rate = max_rate_per_second
        self._current: float = 0.0

    def smooth(self, target: float, dt: float) -> float:
        """Move current value toward target, limited by max rate.

        Args:
            target: Desired value.
            dt: Time step in seconds.

        Returns:
            Smoothed value after applying rate limit.
        """
        max_change = self._max_rate * dt
        diff = target - self._current
        if abs(diff) <= max_change:
            self._current = target
        else:
            self._current += max_change if diff > 0 else -max_change
        return self._current

    def reset(self):
        """Reset the internal state to 0."""
        self._current = 0.0
