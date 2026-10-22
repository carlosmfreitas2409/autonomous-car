"""The cargo compartment, as the cargo node (firmware/cargo) reports it.

The sonar node bridges its serial line onto CAN, and the master forwards each
status frame as a `C` line (arduino.py). A run with no cargo node has no
reading at all, and the mission then behaves exactly as without the
compartment.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CargoReading:
    grams: int
    stable: bool        # the last second of readings within a few grams
    lid_closed: bool
    unlocked: bool      # the lock's coil is powered
    fault: bool         # the scale is not answering: the weight means nothing


_OPEN_S = 10.0  # the operator's "open" holds the lock back this long, unless the lid is lifted first


class ManualOpen:
    """The operator's "open the compartment" (tuner), honoured only while the
    car is e-stopped: the lock is held back until the lid is lifted, for
    `_OPEN_S` at most, and never once the car may move again."""

    def __init__(self):
        self._until: Optional[float] = None

    def request(self, now: float) -> None:
        self._until = now + _OPEN_S

    def update(self, now: float, cargo: Optional[CargoReading], stopped: bool) -> bool:
        """Whether to keep the lock open this frame."""
        if self._until is None:
            return False
        if not stopped or cargo is None or not cargo.lid_closed or now >= self._until:
            self._until = None
            return False
        return True
