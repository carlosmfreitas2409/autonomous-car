import math
from collections import deque
from typing import Optional

_SENSORS = 3

_SLOW_CM = (100.0, 150.0, 100.0)

_STOP_CM = (60.0, 75.0, 60.0)

_RELEASE_CM = tuple(stop + 15.0 for stop in _STOP_CM)
_RELEASE_S = 0.5

_MEDIAN_OF = 3
_ECHO_HOLD_S = 0.5


def _median(values) -> Optional[float]:
    ordered = sorted(values, key=lambda d: math.inf if d is None else d)
    return ordered[len(ordered) // 2] if ordered else None


def _ramp(distance: Optional[float], slow_cm: float, stop_cm: float) -> float:
    if distance is None or distance >= slow_cm:
        return 1.0
    return max(0.0, (distance - stop_cm) / (slow_cm - stop_cm))


class ObstacleGuard:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self._history = [deque(maxlen=_MEDIAN_OF) for _ in range(_SENSORS)]
        self._held: list[Optional[float]] = [None] * _SENSORS
        self._held_age = [0.0] * _SENSORS
        self._seen = False
        self._clear_time = 0.0
        self.stale = False
        self.stopped = False
        self.scale = 1.0
        self.distances: tuple[Optional[float], ...] = (None,) * _SENSORS

    @property
    def label(self) -> str:
        if not self._seen:
            return "no data"
        if self.stale:
            return "stale"
        if self.stopped:
            return "stop"
        return "slow" if self.scale < 1.0 else "clear"

    @property
    def nearest_cm(self) -> Optional[float]:
        echoes = [d for d, slow in zip(self.distances, _SLOW_CM)
                  if d is not None and (self.stopped or d < slow)]
        return min(echoes) if echoes else None

    def observe(self, reading) -> None:
        self._seen = True
        self.stale = reading.state == "X"
        distances = tuple(reading.distances)
        if len(distances) != _SENSORS:
            distances = (None, reading.nearest_cm, None)
        for history, distance in zip(self._history, distances):
            history.append(distance)

    def update(self, dt: float) -> float:
        if not self._seen:
            self.scale = 1.0
            return self.scale

        distances = []
        for i, history in enumerate(self._history):
            d = _median(history)
            if d is not None:
                self._held[i], self._held_age[i] = d, 0.0
            elif self._held[i] is not None:
                self._held_age[i] += dt
                if self._held_age[i] <= _ECHO_HOLD_S:
                    d = self._held[i]
                else:
                    self._held[i] = None
            distances.append(d)
        self.distances = tuple(distances)
        blocked = any(d is not None and d <= stop for d, stop in zip(self.distances, _STOP_CM))
        clear = all(d is None or d > release for d, release in zip(self.distances, _RELEASE_CM))

        if self.stale:
            self.stopped = True
            self._clear_time = 0.0
        elif self.stopped:
            if clear:
                self._clear_time += dt
                if self._clear_time >= _RELEASE_S:
                    self.stopped = False
            else:
                self._clear_time = 0.0
        if not self.stopped and blocked:
            self.stopped = True
            self._clear_time = 0.0

        if self.stopped:
            self.scale = 0.0
        else:
            self.scale = min(_ramp(d, slow, stop) for d, slow, stop in zip(self.distances, _SLOW_CM, _STOP_CM))
        return self.scale
