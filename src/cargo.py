from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CargoReading:
    grams: int
    stable: bool
    lid_closed: bool
    unlocked: bool
    fault: bool


_OPEN_S = 10.0


class ManualOpen:
    def __init__(self):
        self._until: Optional[float] = None

    def request(self, now: float) -> None:
        self._until = now + _OPEN_S

    def update(self, now: float, cargo: Optional[CargoReading], stopped: bool) -> bool:
        if self._until is None:
            return False
        if not stopped or cargo is None or not cargo.lid_closed or now >= self._until:
            self._until = None
            return False
        return True
