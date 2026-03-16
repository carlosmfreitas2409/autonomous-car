"""Emergency stop manager for the vehicle safety system."""

from enum import Enum, auto


class _State(Enum):
    RUNNING = auto()
    STOPPED = auto()


class EStopManager:
    """Manages the emergency stop state machine.

    The vehicle starts in RUNNING state. When activated, it transitions
    to STOPPED and all motor output is zeroed. An explicit reset is
    required to resume operation.
    """

    def __init__(self):
        self._state = _State.RUNNING

    def activate(self) -> None:
        """Activate the emergency stop (transition to STOPPED)."""
        self._state = _State.STOPPED

    def reset(self) -> None:
        """Reset the emergency stop (transition to RUNNING)."""
        self._state = _State.RUNNING

    def is_active(self) -> bool:
        """Return True when the emergency stop is active (STOPPED)."""
        return self._state is _State.STOPPED
