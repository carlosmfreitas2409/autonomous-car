"""Turning confirmed sign tracks into a speed constraint.

The policy constrains the controller rather than replacing it: it returns a
throttle ceiling and an optional hold, and the caller applies
`throttle = min(throttle, cap)`. Steering and lane following are never touched.
The previous implementation overwrote throttle with zero, which on a rolling
cart is a coast, and only in autonomous mode.

Two things here exist because of how the track is physically laid out:

  The latch      With a forward-facing camera and a sign at the roadside, a
                 stop sign leaves the frame *before* the cart reaches the line.
                 A policy that only reacts to what it can currently see would
                 accelerate away the moment the sign disappears. So once a stop
                 is committed to, the approach continues on dead reckoning.

  Consumption    A sign that has been stopped at is remembered by track ID and
                 never triggers again. The previous version re-armed on a timer
                 and could stop twice at the same sign.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import logging

from perception.sign_detection import INFORMATIONAL_LABELS
from perception.sign_worker import SignSnapshot
from perception.tracking import Track
from perception.traffic_light import LightState

logger = logging.getLogger(__name__)


# Where the approach begins and where the cart must be stopped, in metres.
_SLOW_DISTANCE_M = 3.0
_STOP_DISTANCE_M = 0.6

# Lowest throttle that still moves the cart. Below this it stalls rather than
# creeps, so the deceleration ramp bottoms out here instead of at zero.
_CREEP_THROTTLE = 0.30

# Time held stationary at a stop sign, plus the slack at the start of it for
# the cart to actually coast to rest — throttle zero is not an instant stop.
_DWELL_S = 3.0
_SETTLE_S = 0.5

# Rough metres per second at full throttle. One measurement (drive at a known
# throttle for a few seconds, measure the distance) makes the deceleration
# ramp and the dead reckoning quantitative.
_SPEED_PER_THROTTLE = 1.2

# Detections older than this are never acted on. Perception that has stopped
# publishing must not be able to steer behaviour with a stale frame.
_STALE_S = 0.4

# How long a latched approach survives with no sighting at all before it is
# abandoned as a false alarm. Comfortably longer than the couple of seconds it
# takes to cover the approach distance.
_LATCH_MEMORY_S = 6.0

# Ceiling on any hold. A cart frozen forever on a light it can no longer read
# is the worst outcome available here, so the hold always ends eventually.
_MAX_HOLD_S = 20.0

# How long a consumed stop sign stays consumed. Only garbage collection: the
# real protection is the track ID, which is never reused. On a later lap the
# same physical sign arrives as a new track and correctly stops the cart again.
_CONSUMED_MEMORY_S = 30.0

# After releasing from a stop, ignore stop signs for this long. "Consumed"
# works by track ID, so it cannot recognise a sign that briefly dropped out of
# view during the halt and came back with a fresh ID. This covers that window,
# and at creep speed it is well under the gap between two real signs.
_REARM_DELAY_S = 2.0

_HALT_LIGHT_STATES = (LightState.RED, LightState.YELLOW)


class PolicyState(Enum):
    CLEAR = "clear"
    SLOWING = "slowing"
    HOLDING = "holding"
    DWELLING = "dwelling"
    WAITING_GREEN = "waiting_green"


@dataclass
class TrafficConstraint:
    speed_cap: float = 1.0
    hold: bool = False
    state: PolicyState = PolicyState.CLEAR
    reason: str = ""
    stale: bool = False
    # Reported, never acted on. The routing layer decides what a detour or a
    # pickup marker means; perception's job is only to say it is there and how
    # far away. `pickup` is the nearest of whichever markers are in view - its
    # .label says which one.
    detour: Optional[Track] = None
    pickup: Optional[Track] = None


@dataclass
class _Limit:
    """One rule's demand, before they are combined."""

    cap: float = 1.0
    hold: bool = False
    state: PolicyState = PolicyState.CLEAR
    reason: str = ""


class TrafficPolicy:
    def __init__(self):
        self._stop_latch: Optional[_Approach] = None
        self._light_gate: Optional[_Approach] = None
        self._consumed: dict[int, float] = {}
        self._stop_released_at = float("-inf")

    def reset(self) -> None:
        self._stop_latch = None
        self._light_gate = None
        self._consumed.clear()
        self._stop_released_at = float("-inf")

    def update(self, snapshot: SignSnapshot, throttle: float, dt: float, now: float) -> TrafficConstraint:
        stale = snapshot.age(now) > _STALE_S
        tracks = [] if stale else [t for t in snapshot.tracks if t.confirmed]
        travelled = max(0.0, throttle) * _SPEED_PER_THROTTLE * dt

        self._expire_consumed(now)

        stop_limit = self._update_stop(tracks, travelled, now)
        light_limit = self._update_light(tracks, travelled, now, stale)

        # Most restrictive rule wins: a hold beats any cap, and otherwise the
        # lower ceiling applies.
        if stop_limit.hold:
            limit = stop_limit
        elif light_limit.hold:
            limit = light_limit
        else:
            limit = min(stop_limit, light_limit, key=lambda candidate: candidate.cap)

        return TrafficConstraint(
            speed_cap=limit.cap,
            hold=limit.hold,
            state=limit.state,
            reason=limit.reason,
            stale=stale,
            detour=_nearest(tracks, "detour"),
            pickup=_nearest_of(tracks, INFORMATIONAL_LABELS),
        )

    def _update_stop(self, tracks: list[Track], travelled: float, now: float) -> _Limit:
        sighting = _nearest(tracks, "stop", exclude=set(self._consumed))

        if self._stop_latch is None:
            if sighting is None or not _within_range(sighting):
                return _Limit()
            if now - self._stop_released_at < _REARM_DELAY_S:
                return _Limit()
            self._stop_latch = _Approach(track_id=sighting.id, distance_m=sighting.distance_m, last_seen=now)
            logger.info("Stop sign %d committed at %.2f m", sighting.id, sighting.distance_m)

        latch = self._stop_latch
        latch.advance(sighting, travelled, now)

        if latch.abandoned(now):
            logger.info("Stop sign %d lost before the stop line - releasing", latch.track_id)
            self._stop_latch = None
            return _Limit()

        if not latch.holding:
            if latch.distance_m > _STOP_DISTANCE_M:
                return _Limit(cap=_approach_cap(latch.distance_m), state=PolicyState.SLOWING, reason="stop sign ahead")
            latch.begin_hold(now)

        held = now - latch.hold_started
        if held >= _SETTLE_S + _DWELL_S or held >= _MAX_HOLD_S:
            self._consumed[latch.track_id] = now
            self._stop_latch = None
            self._stop_released_at = now
            logger.info("Stop sign %d cleared after %.1f s", latch.track_id, held)
            return _Limit()

        state = PolicyState.HOLDING if held < _SETTLE_S else PolicyState.DWELLING
        return _Limit(cap=0.0, hold=True, state=state, reason="stopped at sign")

    def _update_light(self, tracks: list[Track], travelled: float, now: float, stale: bool) -> _Limit:
        if stale:
            # A gate that can only be opened by seeing green must not stay shut
            # once frames have stopped arriving, because then it never can be
            # opened - it would simply sit there until the maximum hold expired.
            # A stop-sign latch is the opposite case and deliberately does
            # survive this: it completes on its own and releases itself.
            if self._light_gate is not None:
                logger.warning("Sign perception went stale at light %d - releasing", self._light_gate.track_id)
                self._light_gate = None
            return _Limit()

        sighting = _nearest(tracks, "traffic_light")

        if self._light_gate is None:
            if sighting is None or not _within_range(sighting):
                return _Limit()
            if sighting.distance_m <= _STOP_DISTANCE_M:
                # Already at or past the line with nothing committed, so the
                # cart was waved through. A light changing now cannot be
                # stopped for without halting on top of it - commit and clear.
                return _Limit()
            if sighting.light_state not in _HALT_LIGHT_STATES:
                # Green, or not yet readable at this distance. Nothing to do.
                return _Limit() if sighting.light_state is LightState.GREEN else _unreadable_limit(sighting)
            self._light_gate = _Approach(track_id=sighting.id, distance_m=sighting.distance_m, last_seen=now)
            logger.info("Traffic light %d showing %s at %.2f m", sighting.id, sighting.light_state.value, sighting.distance_m)

        gate = self._light_gate
        gate.advance(sighting, travelled, now)

        # Green releases the gate outright, at any point in the approach.
        if sighting is not None and sighting.light_state is LightState.GREEN:
            self._light_gate = None
            logger.info("Traffic light %d green - proceeding", gate.track_id)
            return _Limit()

        if gate.abandoned(now):
            logger.info("Traffic light %d lost - releasing", gate.track_id)
            self._light_gate = None
            return _Limit()

        if not gate.holding:
            if gate.distance_m > _STOP_DISTANCE_M:
                return _Limit(cap=_approach_cap(gate.distance_m), state=PolicyState.SLOWING, reason="red light ahead")
            gate.begin_hold(now)

        if now - gate.hold_started >= _MAX_HOLD_S:
            logger.warning(
                "Traffic light %d held for %.0f s without a green - releasing under the maximum hold",
                gate.track_id, _MAX_HOLD_S,
            )
            self._light_gate = None
            return _Limit()

        # An unreadable light while stopped keeps the cart stopped. Only a
        # confirmed green gets it moving.
        return _Limit(cap=0.0, hold=True, state=PolicyState.WAITING_GREEN, reason="waiting for green")

    def _expire_consumed(self, now: float) -> None:
        self._consumed = {
            track_id: at for track_id, at in self._consumed.items() if now - at <= _CONSUMED_MEMORY_S
        }


@dataclass
class _Approach:
    """A committed approach to one object, surviving loss of sight."""

    track_id: int
    # Never None: an approach is only ever created from a sighting that
    # already had a distance.
    distance_m: float
    last_seen: float
    holding: bool = False
    hold_started: float = 0.0

    def advance(self, sighting: Optional[Track], travelled: float, now: float) -> None:
        if sighting is not None and sighting.id == self.track_id and sighting.distance_m is not None:
            self.distance_m = sighting.distance_m
            self.last_seen = now
        elif not self.holding:
            # Out of sight: keep closing on it using the distance we know we
            # have covered since the last sighting.
            self.distance_m = max(0.0, self.distance_m - travelled)

    def abandoned(self, now: float) -> bool:
        return not self.holding and now - self.last_seen > _LATCH_MEMORY_S

    def begin_hold(self, now: float) -> None:
        self.holding = True
        self.hold_started = now


def _approach_cap(distance_m: float) -> float:
    """Throttle ceiling that falls linearly from the slow point to the line."""
    span = _SLOW_DISTANCE_M - _STOP_DISTANCE_M
    progress = max(0.0, min(1.0, (distance_m - _STOP_DISTANCE_M) / span))
    return _CREEP_THROTTLE + progress * (1.0 - _CREEP_THROTTLE)


def _unreadable_limit(sighting: Track) -> _Limit:
    """A light in range whose state we cannot read yet: slow, keep looking."""
    return _Limit(
        cap=_approach_cap(sighting.distance_m),
        state=PolicyState.SLOWING,
        reason="unreadable light ahead",
    )


def _within_range(track: Track) -> bool:
    return track.distance_m is not None and track.distance_m <= _SLOW_DISTANCE_M


def _nearest(tracks: list[Track], label: str, exclude: set[int] = frozenset()) -> Optional[Track]:
    return _nearest_of(tracks, (label,), exclude)


def _nearest_of(tracks: list[Track], labels, exclude: set[int] = frozenset()) -> Optional[Track]:
    candidates = [
        t for t in tracks
        if t.label in labels and t.id not in exclude and t.distance_m is not None
    ]
    return min(candidates, key=lambda t: t.distance_m) if candidates else None
