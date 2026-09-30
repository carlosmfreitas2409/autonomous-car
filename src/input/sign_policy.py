"""Confirmed signs and lights become one number: a throttle scale.

Same shape as `Autonomous.obstacle_scale`: 1.0 leaves the car alone, 0.0
stops it, and the ramp between them runs from 3 m down to 2 m. On top of the
scale there is `hold`, which the loop uses to freeze steering while stopped.

Two rules keep this from ever trapping the car or stopping it twice:

  * every hold ends - a stop dwells for a fixed time, a light gate has a 10 s
    ceiling and dies with a stale snapshot;
  * a sign is consumed by track id once it has been acted on, and for 3 s
    after any release nothing close is allowed to start a new one. Ids are
    never reused, so a consumed id cannot come back.

The policy never steers, and it never acts on a detour or delivery sign; it
only reports them.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import logging

from perception.sign_tracker import CAUTION_STATES, HALT_STATES, Track
from perception.sign_worker import SignSnapshot
from perception.traffic_light import LightState

logger = logging.getLogger(__name__)


_SLOW_DISTANCE_M = 3.0
_STOP_DISTANCE_M = 2.0
_SETTLE_S = 0.5
_DWELL_S = 3.0
_MAX_HOLD_S = 10.0          # every hold ends
_MAX_APPROACH_S = 8.0       # an approach that never reaches the line stops where it is
_STALE_S = 0.4              # a snapshot older than this never acts
_LOST_S = 1.0               # a committed approach whose track vanishes this long is abandoned
_LOST_WAITING_S = 2.0       # a light gate whose track vanishes this long while waiting releases
_REARM_S = 3.0              # after any release, ignore that class this long (same sign still in view)
_REARM_DISTANCE_M = 2.5     # ...but only things this close; a second sign further up is real
_PASS_DISTANCE_M = 0.8      # a light first seen closer than this is under the car already: waved through


class PolicyState(Enum):
    CLEAR = "clear"
    SLOWING = "slowing"
    HOLDING = "holding"
    DWELLING = "dwelling"
    WAITING_GREEN = "waiting_green"


@dataclass
class SignConstraint:
    scale: float = 1.0
    hold: bool = False
    state: PolicyState = PolicyState.CLEAR
    reason: str = ""
    stale: bool = False
    distance_m: Optional[float] = None
    detour: Optional[Track] = None      # reported, never acted on
    delivery: Optional[Track] = None    # nearest delivery sign; .variant says A/B/C or None


class _Commitment:
    def __init__(self, track: Track, now: float):
        self.id = track.id
        self.distance = track.distance_m
        self.last_seen = now
        self.committed_at = now
        self.hold_since: Optional[float] = None

    def observe(self, track: Optional[Track], now: float) -> None:
        if track is None:
            return
        self.last_seen = now
        if track.distance_m is not None:
            self.distance = track.distance_m


def _ramp(distance: Optional[float]) -> float:
    if distance is None:
        return 1.0
    span = _SLOW_DISTANCE_M - _STOP_DISTANCE_M
    return max(0.0, min(1.0, (distance - _STOP_DISTANCE_M) / span))


class SignPolicy:
    def __init__(self):
        self._stop: Optional[_Commitment] = None
        self._light: Optional[_Commitment] = None
        self._consumed: set[int] = set()
        self._passed: set[int] = set()
        self._released_at: Optional[float] = None

    def reset(self) -> None:
        """E-stop: drop the active approach, gate and hold. What has been
        consumed stays consumed: an interrupted stop repeats, a completed one
        does not."""
        self._stop = None
        self._light = None

    def update(self, snapshot: SignSnapshot, now: float) -> SignConstraint:
        stale = snapshot.age(now) > _STALE_S
        tracks = [] if stale else list(snapshot.tracks)
        by_id = {track.id: track for track in tracks}
        actionable = [track for track in tracks if track.actionable]

        present = {track.id for track in snapshot.tracks}
        self._consumed &= present
        self._passed &= present

        stop_scale, stop_state, stop_distance = self._update_stop(tracks, by_id, actionable, now)
        light_scale, light_state, light_distance, light_reason = self._update_light(by_id, actionable, stale, now)

        constraint = SignConstraint(stale=stale)
        constraint.detour = _nearest(tracks, "detour")
        constraint.delivery = _nearest(tracks, "delivery_sign")

        # Most restrictive wins: any hold beats a ramp, a lower scale beats a
        # higher one. A red light outranks a stop sign for the caption only.
        held = light_state is PolicyState.WAITING_GREEN or stop_state in (PolicyState.HOLDING, PolicyState.DWELLING)
        if held:
            if light_state is PolicyState.WAITING_GREEN:
                state, distance = light_state, light_distance
            else:
                state, distance = stop_state, stop_distance
            constraint.scale = 0.0
            constraint.hold = True
            constraint.state = state
            constraint.distance_m = distance
            constraint.reason = {
                PolicyState.HOLDING: "stopped at sign",
                PolicyState.DWELLING: "stopped at sign",
                PolicyState.WAITING_GREEN: "waiting for green",
            }[state]
            return constraint

        scale = min(stop_scale, light_scale)
        if PolicyState.SLOWING in (stop_state, light_state):
            constraint.scale = scale
            constraint.state = PolicyState.SLOWING
            if stop_scale <= light_scale:
                constraint.distance_m = stop_distance
                constraint.reason = "stop sign ahead"
            else:
                constraint.distance_m = light_distance
                constraint.reason = light_reason
            return constraint

        if stale:
            constraint.reason = "stale"
        return constraint

    # ------------------------------------------------------------------

    def _update_stop(self, tracks, by_id, actionable, now):
        approach = self._stop
        if approach is not None:
            approach.observe(by_id.get(approach.id), now)
            if approach.hold_since is None:
                if now - approach.last_seen > _LOST_S:
                    # Abandoned: nothing consumed, so a respawned id can commit again.
                    logger.info("stop #%d: lost for %.1f s, abandoned", approach.id, _LOST_S)
                    self._stop = approach = None
                elif approach.distance is not None and approach.distance <= _STOP_DISTANCE_M:
                    approach.hold_since = now
                    logger.info("stop #%d: at the line (%.2f m), holding", approach.id, approach.distance)
                elif now - approach.committed_at > _MAX_APPROACH_S:
                    # Never reached the line: the car has stopped short or the
                    # distance is biased. Stop here rather than crawl forever.
                    approach.hold_since = now
                    logger.warning("stop #%d: %.0f s approaching, still %.2f m - holding here",
                                   approach.id, _MAX_APPROACH_S, approach.distance or -1.0)
            if approach is not None and approach.hold_since is not None:
                held = now - approach.hold_since
                if held >= _SETTLE_S + _DWELL_S:
                    # The sign still in view is this sign, confirmed or not.
                    self._consumed |= {t.id for t in tracks if t.label == "stop"}
                    self._consumed.add(approach.id)
                    self._released_at = now
                    self._stop = None
                    logger.info("stop #%d: released, consumed %s", approach.id, sorted(self._consumed))
                    return 1.0, PolicyState.CLEAR, None
                state = PolicyState.HOLDING if held < _SETTLE_S else PolicyState.DWELLING
                return 0.0, state, approach.distance

        if self._stop is None:
            candidates = [
                t for t in actionable
                if t.label == "stop" and t.id not in self._consumed and t.distance_m is not None
                and t.distance_m <= _SLOW_DISTANCE_M and not self._rearm_blocked(t, now)
            ]
            if candidates:
                nearest = min(candidates, key=lambda t: t.distance_m)
                self._stop = _Commitment(nearest, now)
                logger.info("stop #%d: committed at %.2f m", nearest.id, nearest.distance_m)
                if nearest.distance_m <= _STOP_DISTANCE_M:
                    self._stop.hold_since = now
                    return 0.0, PolicyState.HOLDING, nearest.distance_m

        if self._stop is None:
            return 1.0, PolicyState.CLEAR, None
        return _ramp(self._stop.distance), PolicyState.SLOWING, self._stop.distance

    def _rearm_blocked(self, track: Track, now: float) -> bool:
        if self._released_at is None or now - self._released_at >= _REARM_S:
            return False
        return track.distance_m is not None and track.distance_m <= _REARM_DISTANCE_M

    def _update_light(self, by_id, actionable, stale, now):
        gate = self._light
        if gate is not None:
            track = by_id.get(gate.id)
            gate.observe(track, now)
            if track is not None and track.light_state is LightState.GREEN:
                # Cleared, nothing else: it may re-gate if it goes red again.
                logger.info("light #%d: GREEN, gate cleared", gate.id)
                self._light = gate = None
            elif stale:
                logger.warning("light #%d: snapshot stale, gate released", gate.id)
                self._release_light(gate, now)
                gate = None
            elif gate.hold_since is None:
                if now - gate.last_seen > _LOST_S:
                    logger.info("light #%d: lost for %.1f s, gate abandoned", gate.id, _LOST_S)
                    self._light = gate = None
                elif gate.distance is not None and gate.distance <= _STOP_DISTANCE_M:
                    gate.hold_since = now
                    logger.info("light #%d: at the line (%.2f m), waiting for green", gate.id, gate.distance)
                elif now - gate.committed_at > _MAX_APPROACH_S:
                    gate.hold_since = now
                    logger.warning("light #%d: %.0f s approaching, still %.2f m - waiting here",
                                   gate.id, _MAX_APPROACH_S, gate.distance or -1.0)
            if gate is not None and gate.hold_since is not None:
                if (now - gate.last_seen > _LOST_WAITING_S
                        or now - gate.hold_since >= _MAX_HOLD_S):
                    logger.warning("light #%d: released without green (%s)", gate.id,
                                   "lost" if now - gate.last_seen > _LOST_WAITING_S else "ceiling")
                    self._release_light(gate, now)
                    return 1.0, PolicyState.CLEAR, None, ""
                return 0.0, PolicyState.WAITING_GREEN, gate.distance, "waiting for green"

        if self._light is None:
            for track in actionable:
                if track.label != "traffic_light" or track.distance_m is None:
                    continue
                if track.id in self._passed or track.id in self._consumed:
                    continue
                if track.distance_m <= _PASS_DISTANCE_M:
                    # Braking would put the car under the light: waved through.
                    logger.info("light #%d: %s first seen at %.2f m, waved through",
                                track.id, track.light_state.value, track.distance_m)
                    self._passed.add(track.id)
            # A curve can reveal the light inside the line. A red is a red:
            # the car stops late rather than not at all.
            candidates = [
                t for t in actionable
                if t.label == "traffic_light" and t.light_state in HALT_STATES
                and t.id not in self._passed and t.id not in self._consumed
                and t.distance_m is not None and _PASS_DISTANCE_M < t.distance_m <= _SLOW_DISTANCE_M
            ]
            if candidates:
                nearest = min(candidates, key=lambda t: t.distance_m)
                self._light = _Commitment(nearest, now)
                logger.info("light #%d: %s at %.2f m, gate formed",
                            nearest.id, nearest.light_state.value, nearest.distance_m)
                if nearest.distance_m <= _STOP_DISTANCE_M:
                    self._light.hold_since = now
                    logger.info("light #%d: already inside the line, waiting for green", nearest.id)
                    return 0.0, PolicyState.WAITING_GREEN, nearest.distance_m, "waiting for green"

        if self._light is not None:
            return _ramp(self._light.distance), PolicyState.SLOWING, self._light.distance, "red light ahead"

        # Yellow: slow down, keep rolling. The loop floors the throttle at the
        # crawl speed, so this never stops the car on its own.
        caution = [
            t for t in actionable
            if t.label == "traffic_light" and t.light_state in CAUTION_STATES
            and t.id not in self._passed and t.id not in self._consumed
            and t.distance_m is not None and t.distance_m <= _SLOW_DISTANCE_M
        ]
        if caution:
            nearest = min(caution, key=lambda t: t.distance_m)
            return _ramp(nearest.distance_m), PolicyState.SLOWING, nearest.distance_m, "yellow light ahead"
        return 1.0, PolicyState.CLEAR, None, ""

    def _release_light(self, gate: _Commitment, now: float) -> None:
        self._consumed.add(gate.id)
        self._released_at = now
        self._light = None


def _nearest(tracks: list[Track], label: str) -> Optional[Track]:
    matching = [t for t in tracks if t.label == label and t.confirmed]
    if not matching:
        return None
    return min(matching, key=lambda t: t.distance_m if t.distance_m is not None else float("inf"))
