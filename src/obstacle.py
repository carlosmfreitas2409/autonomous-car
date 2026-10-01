"""Front sonar guard: when to slow and when to stop.

The three HC-SR04s look left, ahead and right. Every sensor slows the car on
the way to the stop distance, so it stops gently whichever one sees the
obstacle. The side ones point outward and also see cones, walls and people
beside the track, so they start slowing later than the centre one: a side
echo farther than `_SLOW_CM` for that sensor is ignored.

Stopping is a latch so a stationary obstacle cannot make the car stop and go:
it stops at `_STOP_CM` and only resumes once every sensor reads farther than
`_RELEASE_CM` for `_RELEASE_S`. Each sensor is also median-filtered over its
last three readings, so one ghost echo from a neighbouring sensor's ping
cannot slow or stop the car, and a distance it did measure is held for
`_ECHO_HOLD_S` through missing echoes: an obstacle at the edge of a side
sensor's beam only echoes on some pings, and forgetting it between them sped
the car back up to full.

The guard makes no decision about the mode: it runs every frame so the GUI
shows what it sees, and the autonomous controller is the only consumer of
`scale`.
"""

import math
from collections import deque
from typing import Optional

_SENSORS = 3

# Where each sensor starts slowing the car, left to right. The sides start
# later because they see things beside the track (a person standing off it
# read ~80 cm); lower them if that still slows the car, raise them if it
# reaches the stop too fast.
_SLOW_CM = (100.0, 150.0, 100.0)
_STOP_CM = 60.0

# Once stopped, the car waits until every sensor reads farther than
# `_RELEASE_CM` for `_RELEASE_S`: an obstacle standing at the threshold reads
# a few cm either side of it from one ping to the next.
_RELEASE_CM = _STOP_CM + 15.0
_RELEASE_S = 0.5

_MEDIAN_OF = 3
_ECHO_HOLD_S = 0.5


def _median(values) -> Optional[float]:
    # A missing echo sorts as farthest; with an even count the upper middle
    # is taken, so a single ghost at start-up cannot stop the car.
    ordered = sorted(values, key=lambda d: math.inf if d is None else d)
    return ordered[len(ordered) // 2] if ordered else None


def _ramp(distance: Optional[float], slow_cm: float) -> float:
    if distance is None or distance >= slow_cm:
        return 1.0
    return max(0.0, (distance - _STOP_CM) / (slow_cm - _STOP_CM))


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
        # Filtered distance per sensor, left to right (None = nothing in
        # range), for the GUI and the log.
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
        """The nearest echo inside its sensor's slow distance (or, while
        stopped, the nearest at all)."""
        echoes = [d for d, slow in zip(self.distances, _SLOW_CM)
                  if d is not None and (self.stopped or d < slow)]
        return min(echoes) if echoes else None

    def observe(self, reading) -> None:
        """Feed a new sonar line from the master (call once per line, not per frame)."""
        self._seen = True
        self.stale = reading.state == "X"
        distances = tuple(reading.distances)
        if len(distances) != _SENSORS:
            # A master that only reports the nearest echo: treat it as dead ahead.
            distances = (None, reading.nearest_cm, None)
        for history, distance in zip(self._history, distances):
            history.append(distance)

    def update(self, dt: float) -> float:
        """Throttle scale for this frame: 1.0 clear, 0.0 stopped, a ramp between."""
        if not self._seen:
            # No master on the serial port (a replay): nothing to guard with.
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
        nearest = min(math.inf if d is None else d for d in self.distances)

        if self.stale:
            # No CAN frame from the sonar node for 300 ms: blocked, never clear.
            self.stopped = True
            self._clear_time = 0.0
        elif self.stopped:
            if nearest > _RELEASE_CM:
                self._clear_time += dt
                if self._clear_time >= _RELEASE_S:
                    self.stopped = False
            else:
                self._clear_time = 0.0
        if not self.stopped and nearest <= _STOP_CM:
            self.stopped = True
            self._clear_time = 0.0

        if self.stopped:
            self.scale = 0.0
        else:
            self.scale = min(_ramp(d, slow) for d, slow in zip(self.distances, _SLOW_CM))
        return self.scale
