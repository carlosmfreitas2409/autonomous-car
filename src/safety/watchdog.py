"""Command watchdog timer for the vehicle safety system."""

import time

from src.config import WATCHDOG_TIMEOUT


class CommandWatchdog:
    """Monitors command freshness and triggers safety action on timeout.

    If no command is received (via ``feed()``) within the configured
    timeout window, ``is_expired()`` returns True, signalling that
    the safety system should engage the emergency stop.
    """

    def __init__(self, timeout: float = WATCHDOG_TIMEOUT):
        self._timeout = timeout
        self._last_feed: float = time.monotonic()

    def feed(self) -> None:
        """Record receipt of a fresh command."""
        self._last_feed = time.monotonic()

    def is_expired(self) -> bool:
        """Return True if time since last feed exceeds the timeout."""
        return (time.monotonic() - self._last_feed) > self._timeout
