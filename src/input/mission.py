"""Pickup and delivery: which way to go at each detour sign, and where to stop.

The track is a loop with two exits a few metres apart: exit 1 (A inside its
detour), then exit 2 (B inside its detour), then C on the main road, and round
again. Exit 2's mouth sits just past exit 1's, so detour 1 rejoins the road
beyond it: exit 2 is only reached by going straight at exit 1, and every other
way leads round to exit 1 again. Both detour signs look alike and nothing
marks a new lap, so the mission counts: `next_exit` starts at 1 (the car starts
before exit 1) and moves on at every detour sign by `_exit_after`.

At a detour sign the rule is local: enter only if the current target - the
pickup until it has been collected, then the delivery - is the letter inside
this exit. On a fixed cycle, taking the first chance at each target in order
is already the shortest route, so there is no route table.

A letter read up close is the one absolute position fix there is: A means
detour 1, B means detour 2, C means the main road after exit 2 - and from all
three the next exit is exit 1. Each letter sign resets `next_exit` once, so a
missed or double-counted detour sign costs one wrong detour or one extra lap,
never a mission that is lost for good.

Nothing here acts on a single frame or on a far sign: only confirmed tracks
(3 of the last 5 frames, sign_tracker.py) with a committed letter (3 of the
last 5 readings) count, and only within `_ACT_DISTANCE_M`.

Like `SignPolicy`, the mission never steers: it returns a throttle scale, a
hold, a one-shot path request and an optional turn signal, and the loop
applies them. With no letters configured it returns the neutral command and
the car drives exactly as before.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import logging

from lights import TurnSignal
from path import Path
from perception.sign_tracker import Track
from perception.sign_worker import SignSnapshot

logger = logging.getLogger(__name__)


LETTERS = ("A", "B", "C")

# Which exit's detour holds each letter; C is on the main road.
LETTER_EXIT = {"A": 1, "B": 2, "C": None}



def _exit_after(exit_number: int, entered: bool) -> int:
    """The exit met next. Detour 1 rejoins the road past exit 2's mouth
    (live 2026-10-07: after A the next sign was exit 1's again), so only
    going straight at exit 1 reaches exit 2."""
    return 2 if exit_number == 1 and not entered else 1


# The next exit implied by reading a letter: inside detour 1, inside detour 2
# and at C alike, the road leads round to exit 1.
_EXIT_AFTER_LETTER = {"A": _exit_after(1, True), "B": _exit_after(2, True), "C": 1}

# Same distances as the stop sign: nothing further than this is acted on, and
# the approach ramps from here down to a stop at _STOP_DISTANCE_M.
_ACT_DISTANCE_M = 3.0
_STOP_DISTANCE_M = 1.5
_SETTLE_S = 0.5
_DWELL_S = 3.0              # pickup only: the delivery stop never ends
_LOST_S = 1.0               # a committed approach whose track vanishes this long is abandoned
_MAX_APPROACH_S = 8.0       # an approach that never reaches the line stops where it is
_STALE_S = 0.4              # a snapshot older than this decides nothing
# A detour sign that turns up under a new track id is the one already decided
# - the tracker lost it and found it again - if that one was seen this
# recently and the new one is no farther than it was. Two exits can sit a few
# metres apart, with both signs in view at once: the second is farther away.
_SAME_SIGN_S = 2.0
_SAME_SIGN_M = 0.5
_SIGNAL_BEFORE_LEAVING_S = 2.0  # pickup: left signal on, still stopped, before pulling away
_LEAVING_S = 2.0            # ...and kept on this long after pulling away


class Stage(Enum):
    IDLE = "idle"
    PICKUP = "pickup"
    DELIVERY = "delivery"
    DONE = "done"


class MissionState(Enum):
    IDLE = "idle"
    DRIVING = "driving"
    SLOWING = "slowing"
    STOPPED = "stopped"
    LEAVING = "leaving"
    DONE = "done"


@dataclass
class MissionCommand:
    scale: float = 1.0
    hold: bool = False
    path: Optional[Path] = None                 # one-shot request, applied by the loop
    turn_signal: Optional[TurnSignal] = None    # None: no opinion
    state: MissionState = MissionState.IDLE
    reason: str = ""


class _Approach:
    def __init__(self, track: Track, now: float):
        self.id = track.id
        self.letter = track.variant
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


def plan_route(pickup: str, delivery: str) -> list[str]:
    """The decisions the mission expects to make, in order, from the start."""
    steps: list[str] = []
    next_exit, c_ahead = 1, False
    for target, role in ((pickup, "pickup"), (delivery, "delivery")):
        if role == "delivery" and target == pickup:
            break
        while True:
            # A letter outside every detour (C) is on the main road after exit 2.
            if LETTER_EXIT[target] is None and c_ahead:
                steps.append(f"{target}: {role}")
                c_ahead = False
                break
            exit_number = next_exit
            enter = LETTER_EXIT[target] == exit_number
            steps.append(f"exit {exit_number}: {'detour' if enter else 'straight'}")
            # Every way back round to exit 1 passes C.
            next_exit = _exit_after(exit_number, enter)
            c_ahead = next_exit == 1
            if enter:
                steps.append(f"{target}: {role}")
                break
    return steps


class Mission:
    def __init__(self):
        self._pickup: Optional[str] = None
        self._delivery: Optional[str] = None
        self.route: list[str] = []
        self._restart(Stage.IDLE)

    def _restart(self, stage: Stage) -> None:
        self.stage = stage
        self.next_exit = 1
        self._approach: Optional[_Approach] = None
        self._consumed: set[int] = set()
        self._fixed: set[int] = set()
        self._decided: set[int] = set()
        # The last decided detour sign: its ids, nearest distance, last seen.
        self._sign_ids: set[int] = set()
        self._sign_distance = float("inf")
        self._sign_seen = float("-inf")
        # Exits decided straight whose mouth the lane camera has not shown
        # yet, and a Detour decided while one of them is still ahead.
        self._forks_ahead = 0
        self._pending_exit: Optional[int] = None
        self._leaving_until: Optional[float] = None

    @property
    def pickup(self) -> Optional[str]:
        return self._pickup

    @property
    def delivery(self) -> Optional[str]:
        return self._delivery

    @property
    def target(self) -> Optional[str]:
        if self.stage is Stage.PICKUP:
            return self._pickup
        if self.stage is Stage.DELIVERY:
            return self._delivery
        return None

    def configure(self, pickup: Optional[str], delivery: Optional[str]) -> None:
        """Start a new run from before exit 1, or stay idle while either letter
        is missing. A half-made choice is kept: the operator picks the two
        letters one at a time. The caller decides when a change is allowed."""
        self._pickup = pickup if pickup in LETTERS else None
        self._delivery = delivery if delivery in LETTERS else None
        if self._pickup is None or self._delivery is None:
            self.route = []
            self._restart(Stage.IDLE)
            logger.info("mission: idle (pickup %s, delivery %s)", self._pickup or "-", self._delivery or "-")
            return
        self.route = plan_route(pickup, delivery)
        self._restart(Stage.PICKUP)
        logger.info("mission: pickup %s, delivery %s - %s", pickup, delivery, " -> ".join(self.route))

    def reset(self) -> None:
        """E-stop: drop the approach in progress, so an interrupted pickup stop
        repeats. The stage and the exit count stay: the car has not moved."""
        self._approach = None
        self._leaving_until = None

    def update(self, snapshot: SignSnapshot, path: Path, now: float,
               fork_passed: bool = False) -> MissionCommand:
        """`fork_passed`: `path.ForkWatch` saw the car drive past a detour's
        mouth on the main road since the last call."""
        if self.stage is Stage.IDLE:
            return MissionCommand()
        if self.stage is Stage.DONE:
            return self._done()

        stale = snapshot.age(now) > _STALE_S
        tracks = [] if stale else [t for t in snapshot.tracks if t.confirmed]

        self._consumed &= {t.id for t in snapshot.tracks}
        self._fix_position(tracks)
        self._follow_sign(snapshot, stale, now)
        request = self._follow_forks(fork_passed, now)
        decided = self._decide_detour(tracks, path, now)
        command = self._update_approach(tracks, now)
        command.path = decided or request
        return command

    # ------------------------------------------------------------------

    def _fix_position(self, tracks: list[Track]) -> None:
        # Once per track id (ids are never reused): C still in view as the car
        # reaches exit 1's sign must not undo that sign's count.
        for track in tracks:
            if (track.label != "delivery_sign" or track.variant not in _EXIT_AFTER_LETTER
                    or track.id in self._fixed or not _within(track, _ACT_DISTANCE_M)):
                continue
            self._fixed.add(track.id)
            # Every letter stands past the mouths behind it.
            self._forks_ahead = 0
            fixed = _EXIT_AFTER_LETTER[track.variant]
            if fixed != self.next_exit:
                logger.warning("mission: read %s at %.1f m - next exit %d -> %d",
                               track.variant, track.distance_m, self.next_exit, fixed)
                self.next_exit = fixed

    def _follow_sign(self, snapshot, stale, now) -> None:
        """Keep up with the last decided sign. A stale snapshot counts as it
        still being in view: letting it expire there would decide it twice."""
        for track in snapshot.tracks:
            if track.id in self._sign_ids:
                self._sign_seen = now
                if track.distance_m is not None:
                    self._sign_distance = min(self._sign_distance, track.distance_m)
        if stale:
            self._sign_seen = now

    def _follow_forks(self, fork_passed, now) -> Optional[Path]:
        """Arm a waiting Detour once the mouth before it is behind the car.

        Exits a few metres apart: the second sign is decided while the car is
        still level with the first exit's mouth. Right-edge anchoring there
        would follow the first detour's line, and DetourReturn would take that
        mouth for the entrance and send the path home before the second one.
        """
        if fork_passed and self._forks_ahead:
            self._forks_ahead -= 1
            logger.info("mission: a mouth passed on the main road, %d still ahead", self._forks_ahead)
            if self._pending_exit is not None and self._forks_ahead == 0:
                logger.info("mission: exit %d -> detour (armed)", self._pending_exit)
                self._pending_exit = None
                return Path.DETOUR
        if self._pending_exit is not None and now - self._sign_seen > _LOST_S:
            # Its sign is behind the car and the mouth before it never showed:
            # arming now would come after this exit's own mouth.
            logger.warning("mission: exit %d's sign passed before the mouth before it was seen - "
                           "straight on, next lap", self._pending_exit)
            self._pending_exit = None
            self._forks_ahead = 0
        return None

    def _decide_detour(self, tracks, path, now) -> Optional[Path]:
        # Inside a detour until DetourReturn sends the path home: no sign
        # out here belongs to the next exit yet.
        if path is Path.DETOUR:
            return None

        candidates = [t for t in tracks if t.label == "detour" and t.id not in self._decided
                      and _within(t, _ACT_DISTANCE_M)]
        if not candidates:
            return None
        sign = min(candidates, key=lambda t: t.distance_m)
        self._decided.add(sign.id)

        if (now - self._sign_seen <= _SAME_SIGN_S
                and sign.distance_m <= self._sign_distance + _SAME_SIGN_M):
            logger.info("mission: detour sign #%d at %.1f m is the one already decided (%.1f m)",
                        sign.id, sign.distance_m, self._sign_distance)
            self._sign_ids.add(sign.id)
            return None

        exit_number = self.next_exit
        wanted = Path.DETOUR if LETTER_EXIT[self.target] == exit_number else Path.STRAIGHT
        logger.info("mission: exit %d sign #%d at %.1f m, target %s -> %s",
                    exit_number, sign.id, sign.distance_m, self.target, wanted.value)
        self._sign_ids = {sign.id}
        self._sign_distance = sign.distance_m
        self._sign_seen = now
        self.next_exit = _exit_after(exit_number, wanted is Path.DETOUR)
        self._pending_exit = None

        if wanted is Path.STRAIGHT:
            self._forks_ahead += 1
        elif self._forks_ahead:
            logger.info("mission: exit %d -> detour once the mouth before it is behind (%d ahead)",
                        exit_number, self._forks_ahead)
            self._pending_exit = exit_number
            return None
        return None if path is wanted else wanted

    def _update_approach(self, tracks: list[Track], now: float) -> MissionCommand:
        approach = self._approach
        if approach is not None and approach.hold_since is None:
            approach.observe(next((t for t in tracks if t.id == approach.id), None), now)
            if now - approach.last_seen > _LOST_S:
                logger.info("mission: %s #%d lost for %.1f s, approach abandoned",
                            approach.letter, approach.id, _LOST_S)
                self._approach = approach = None
            elif now - approach.committed_at > _MAX_APPROACH_S:
                logger.warning("mission: %s #%d %.0f s approaching, still %.2f m - stopping here",
                               approach.letter, approach.id, _MAX_APPROACH_S, approach.distance or -1.0)
                self._arrive(approach, now)

        if approach is None:
            candidates = [t for t in tracks if t.label == "delivery_sign" and t.variant == self.target
                          and t.id not in self._consumed and _within(t, _ACT_DISTANCE_M)]
            if candidates:
                nearest = min(candidates, key=lambda t: t.distance_m)
                self._approach = approach = _Approach(nearest, now)
                logger.info("mission: %s #%d at %.2f m, approaching for %s",
                            nearest.variant, nearest.id, nearest.distance_m, self.stage.value)

        if approach is not None and approach.hold_since is None and approach.distance <= _STOP_DISTANCE_M:
            self._arrive(approach, now)

        if self.stage is Stage.DONE:
            return self._done()

        if approach is not None and approach.hold_since is not None:
            held = now - approach.hold_since
            if held < _SETTLE_S + _DWELL_S:
                return MissionCommand(scale=0.0, hold=True, turn_signal=TurnSignal.HAZARD,
                                      state=MissionState.STOPPED, reason=f"pickup at {approach.letter}")
            if held < _SETTLE_S + _DWELL_S + _SIGNAL_BEFORE_LEAVING_S:
                # Signal first, then pull away.
                return MissionCommand(scale=0.0, hold=True, turn_signal=TurnSignal.LEFT,
                                      state=MissionState.LEAVING, reason=f"leaving {approach.letter}")
            self._consumed.add(approach.id)
            self._approach = None
            self.stage = Stage.DELIVERY
            self._leaving_until = now + _LEAVING_S
            logger.info("mission: picked up at %s, delivering to %s", self._pickup, self._delivery)
            approach = None

        if approach is not None:
            return MissionCommand(scale=_ramp(approach.distance), turn_signal=TurnSignal.RIGHT,
                                  state=MissionState.SLOWING,
                                  reason=f"{self.stage.value} at {approach.letter} ahead")

        if self._leaving_until is not None:
            if now < self._leaving_until:
                return MissionCommand(turn_signal=TurnSignal.LEFT, state=MissionState.LEAVING,
                                      reason=f"leaving {self._pickup}")
            self._leaving_until = None
        return MissionCommand(state=MissionState.DRIVING, reason=f"{self.stage.value} at {self.target}")

    def _arrive(self, approach: _Approach, now: float) -> None:
        approach.hold_since = now
        final = self.stage is Stage.DELIVERY or self._pickup == self._delivery
        logger.info("mission: stopped at %s (%.2f m)%s", approach.letter, approach.distance or -1.0,
                    ", delivered" if final else "")
        if final:
            self._approach = None
            self.stage = Stage.DONE

    def _done(self) -> MissionCommand:
        return MissionCommand(scale=0.0, hold=True, turn_signal=TurnSignal.HAZARD,
                              state=MissionState.DONE, reason=f"delivered at {self._delivery}")


def _within(track: Track, distance: float) -> bool:
    return track.distance_m is not None and track.distance_m <= distance


def _ramp(distance: Optional[float]) -> float:
    if distance is None:
        return 1.0
    span = _ACT_DISTANCE_M - _STOP_DISTANCE_M
    return max(0.0, min(1.0, (distance - _STOP_DISTANCE_M) / span))
