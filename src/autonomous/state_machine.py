"""Autonomous driving state machine.

CV Integration:
    The navigator (not yet implemented) should drive transitions by calling:
    - start() when the car begins moving
    - approach_sign() when CV detects a sign ahead
    - stop_at_sign() when the car has stopped in front of the sign
    - update_stop_timer(dt, duration) each frame while stopped (returns True when done)
    - resume() after the stop timer elapses
    - complete() when the mission is finished
"""

from enum import Enum, auto


class AutonomousState(Enum):
    WAITING_FOR_START = auto()
    NAVIGATING = auto()
    APPROACHING_SIGN = auto()
    STOPPED_AT_SIGN = auto()
    ROUTE_COMPLETE = auto()


class AutonomousStateMachine:
    """Manages the autonomous driving behavior states."""

    def __init__(self):
        self._state = AutonomousState.WAITING_FOR_START
        self._stop_timer: float = 0.0

    @property
    def state(self) -> AutonomousState:
        return self._state

    def start(self) -> None:
        if self._state == AutonomousState.WAITING_FOR_START:
            self._state = AutonomousState.NAVIGATING

    def approach_sign(self) -> None:
        if self._state == AutonomousState.NAVIGATING:
            self._state = AutonomousState.APPROACHING_SIGN

    def stop_at_sign(self) -> None:
        if self._state == AutonomousState.APPROACHING_SIGN:
            self._state = AutonomousState.STOPPED_AT_SIGN
            self._stop_timer = 0.0

    def update_stop_timer(self, dt: float, required_duration: float) -> bool:
        """Update the stop timer. Returns True when the required duration has elapsed."""
        if self._state != AutonomousState.STOPPED_AT_SIGN:
            return False
        self._stop_timer += dt
        return self._stop_timer >= required_duration

    def resume(self) -> None:
        if self._state == AutonomousState.STOPPED_AT_SIGN:
            self._state = AutonomousState.NAVIGATING

    def complete(self) -> None:
        self._state = AutonomousState.ROUTE_COMPLETE

    @property
    def stop_elapsed(self) -> float:
        return self._stop_timer
