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

At the target letter the car docks and waits for as long as loading or
unloading takes: only the person at the dock knows when it is done, so they
release the truck (dashboard, tuner or PS5 Cross), as a dock attendant turns
the dock light green for a driver. Released at a pickup, it signals left,
finishes saying it is leaving, and waits for the sonar to read the way ahead
clear for `_DEPART_CLEAR_S` before it pulls away; released at a delivery it parks there, ready for the next
order, which leaves from that dock instead of from the start line.

With the cargo node connected (cargo.py) the compartment is part of the
handshake. It unlocks while the truck waits at a pickup or delivery dock,
until the lid is lifted, and never anywhere else. Closing the lid again is
the release. Before it leaves, the truck checks its own load: the lid closed,
then the weight - something on board after a pickup, nothing after a
delivery. Weight is a check, never the trigger: only the person at the dock
knows when a load is complete. A failed weight check sends the truck back to
waiting, and a second release at that dock overrides it (the attendant knows
best). No cargo node, no checks: the handshake above, unchanged.

Like `SignPolicy`, the mission never steers: it returns a throttle scale, a
hold, a one-shot path request, an optional turn signal, the dock's light scene
and a one-shot announcement, and the loop applies them. With no letters
configured it returns the neutral command and the car drives exactly as
before.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import logging

from cargo import CargoReading
from lights import DockScene, TurnSignal
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
_LOST_S = 1.0               # a committed approach whose track vanishes this long is abandoned
_MAX_APPROACH_S = 8.0       # an approach that never reaches the line stops where it is
_STALE_S = 0.4              # a snapshot older than this decides nothing
# A detour sign that turns up under a new track id is the one already decided
# - the tracker lost it and found it again - if that one was seen this
# recently and the new one is no farther than it was. Two exits can sit a few
# metres apart, with both signs in view at once: the second is farther away.
_SAME_SIGN_S = 2.0
_SAME_SIGN_M = 0.5
# Released, the way ahead must read clear this long - counted once the voice
# has finished - before the car pulls away: the left signal's warning, and
# the attendant's time to step back.
_DEPART_CLEAR_S = 2.0
_LEAVING_S = 2.0            # the left signal kept on this long after pulling away
_REMIND_S = 30.0            # a docked truck repeats what it is waiting for this often
_DELIVERED_S = 3.0          # the "delivered" light before the parked one
# Below this the compartment counts as empty: the scale's drift and the
# vibration of a stopped car stay well under it.
_EMPTY_G = 20
_SETTLE_S = 3.0             # a reading that never settles is judged as it stands after this


class Stage(Enum):
    IDLE = "idle"
    PICKUP = "pickup"
    DELIVERY = "delivery"
    DONE = "done"


class MissionState(Enum):
    IDLE = "idle"
    DRIVING = "driving"
    SLOWING = "slowing"
    DOCKED = "docked"           # waiting for the release
    CHECKING = "checking"       # released, waiting for the way ahead to clear
    LEAVING = "leaving"
    DONE = "done"               # delivered, parked, ready for the next order


@dataclass(frozen=True)
class Announcement:
    key: str        # what happened, see voice.py
    letter: str     # the dock it happened at


@dataclass
class MissionCommand:
    scale: float = 1.0
    hold: bool = False
    path: Optional[Path] = None                 # one-shot request, applied by the loop
    turn_signal: Optional[TurnSignal] = None    # None: no opinion
    state: MissionState = MissionState.IDLE
    reason: str = ""
    scene: Optional[DockScene] = None           # the lights at a dock
    announce: Optional[Announcement] = None     # one-shot, on the frame it happens
    # CHECKING only, what holds the released car: "announcing", "lid",
    # "weighing", "clearing" or "blocked". `reason` says the same in words.
    check: Optional[str] = None


class Dock:
    """Where the car stands waiting for a release. `role` is the stage the
    release completes: PICKUP (loaded), DELIVERY (unloaded), or None for a
    new order leaving from where the last one was delivered."""

    def __init__(self, letter: str, track_id: Optional[int], role: Optional[Stage], now: float):
        self.letter = letter
        self.track_id = track_id
        self.role = role
        self.since = now
        self.released_at: Optional[float] = None
        self.clear_since: Optional[float] = None
        self.announced_at: Optional[float] = None
        self.blocked_announced = False
        # The compartment: lifted since the truck docked or was last
        # released, the weight check that failed (an announcement key), and
        # a release after that failure, which skips the weight from then on.
        self.lid_opened = False
        self.check_failed: Optional[str] = None
        self.override = False
        self.weighing_since: Optional[float] = None
        self.weighed = False
        self.lid_announced = False

    def unrelease(self) -> None:
        self.released_at = None
        self.clear_since = None
        self.blocked_announced = False
        self.weighing_since = None
        self.weighed = False
        self.lid_announced = False


class _Approach:
    def __init__(self, track: Track, now: float):
        self.id = track.id
        self.letter = track.variant
        self.distance = track.distance_m
        self.last_seen = now
        self.committed_at = now

    def observe(self, track: Optional[Track], now: float) -> None:
        if track is None:
            return
        self.last_seen = now
        if track.distance_m is not None:
            self.distance = track.distance_m


def plan_route(pickup: str, delivery: str, start: Optional[str] = None) -> list[str]:
    """The decisions the mission expects to make, in order, from the start
    line, or from the letter `start` where the car is parked."""
    steps: list[str] = []
    next_exit = 1 if start is None else _EXIT_AFTER_LETTER[start]
    # From inside either detour the way back to exit 1 passes C.
    c_ahead = start is not None and LETTER_EXIT[start] is not None
    here = start
    for target, role in ((pickup, "pickup"), (delivery, "delivery")):
        if target == here:
            steps.append(f"{target}: {role}")
            continue
        here = target
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
        # Where the last delivery left the car: (letter, sign track id).
        self._spot: Optional[tuple[str, Optional[int]]] = None
        # The cargo node's latest status; None when it is not connected.
        self._cargo: Optional[CargoReading] = None
        self._quiet = True
        self._restart(Stage.IDLE)

    def _restart(self, stage: Stage) -> None:
        self.stage = stage
        self.next_exit = 1
        self.dock: Optional[Dock] = None
        self._dock_at_spot = False
        self._delivered_at: Optional[float] = None
        self._last_state = MissionState.IDLE
        self.decisions = 0          # detour signs decided since the run started (route progress)
        self.last_decision: Optional[tuple[int, int, Path]] = None  # (sign id, exit, path) of the latest
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
        # The manifest: what the scale read as the truck left its pickup.
        self.manifest_g: Optional[int] = None

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

    @property
    def parked(self) -> bool:
        """Held where the last delivery left it, not yet released: a new order
        can be given without an e-stop, and leaves from here."""
        return self._spot is not None and (self.dock is None or self.dock.released_at is None)

    @property
    def parked_at(self) -> Optional[str]:
        """The letter `parked` holds the car at, else None."""
        return self._spot[0] if self.parked else None

    def configure(self, pickup: Optional[str], delivery: Optional[str]) -> None:
        """Start a new run, or stay idle while either letter is missing. A
        half-made choice is kept: the operator picks the two letters one at a
        time. Parked after a delivery, the run starts at that dock and the car
        stays held until the order is complete; clearing both letters forgets
        the dock (the car may have been carried off). The caller decides when
        a change is allowed."""
        self._pickup = pickup if pickup in LETTERS else None
        self._delivery = delivery if delivery in LETTERS else None
        if self._pickup is None and self._delivery is None:
            self._spot = None
        if self._pickup is None or self._delivery is None:
            self.route = []
            self._restart(Stage.IDLE if self._spot is None else Stage.DONE)
            logger.info("mission: %s (pickup %s, delivery %s)", self.stage.value,
                        self._pickup or "-", self._delivery or "-")
            return
        start = None if self._spot is None else self._spot[0]
        self.route = plan_route(self._pickup, self._delivery, start)
        self._restart(Stage.PICKUP)
        if self._spot is not None:
            # Leaving from this dock: its sign has been read and stays behind.
            self._dock_at_spot = True
            if self._spot[1] is not None:
                self._fixed.add(self._spot[1])
                self._consumed.add(self._spot[1])
        logger.info("mission: pickup %s, delivery %s - %s", self._pickup, self._delivery,
                    " -> ".join(self.route))

    def reset(self) -> None:
        """E-stop: drop the approach in progress and any release, so the car
        needs releasing again. A dock is kept: the car has not moved."""
        self._approach = None
        self._leaving_until = None
        if self.dock is not None:
            self.dock.unrelease()

    def release(self, now: float) -> Optional[str]:
        """The person at the dock says loading or unloading is done. Returns
        why it was refused, or None."""
        if self.dock is None:
            return "not at a dock"
        if self.dock.released_at is not None:
            return "already released"
        self.dock.released_at = now
        # Taken back to waiting, the lid has to be lifted again to release.
        self.dock.lid_opened = False
        if self.dock.check_failed is not None and not self.dock.override:
            self.dock.override = True
            logger.info("mission: released again at %s, leaving without the weight check", self.dock.letter)
        logger.info("mission: released at %s after %.0f s", self.dock.letter, now - self.dock.since)
        return None

    def hold_departure(self) -> Optional[str]:
        """"Wait": a released car that has not left goes back to waiting."""
        if self.dock is None or self.dock.released_at is None:
            return "not leaving a dock"
        self.dock.unrelease()
        logger.info("mission: departure from %s held", self.dock.letter)
        return None

    @property
    def unlock(self) -> bool:
        """The compartment may open: waiting at a pickup or delivery dock,
        until the lid has been lifted (it latches again as it shuts). The loop
        passes it on only while the car cannot move."""
        dock = self.dock
        return (dock is not None and dock.role is not None and dock.released_at is None
                and not dock.lid_opened)

    @property
    def scene(self) -> Optional[DockScene]:
        """The dock's lights while the loop is not asking (e-stopped): the
        people at the dock are still there."""
        if self.dock is not None:
            return DockScene.WAITING
        if self.stage is Stage.DONE and self._spot is not None:
            return DockScene.PARKED
        return None

    def update(self, snapshot: SignSnapshot, path: Path, now: float,
               fork_passed: bool = False, clear: bool = True,
               cargo: Optional[CargoReading] = None, quiet: bool = True) -> MissionCommand:
        """`fork_passed`: `path.ForkWatch` saw the car drive past a detour's
        mouth on the main road since the last call. `clear`: the sonar sees
        nothing ahead; only read while a released car waits to pull away.
        `cargo`: the compartment, None without the cargo node; only read at
        a dock. `quiet`: nothing is being said to the people at the dock."""
        self._cargo = cargo
        self._quiet = quiet
        command = self._update(snapshot, path, now, fork_passed, clear)
        if command.announce is None and command.state is not self._last_state:
            command.announce = self._announce_change(command.state)
        self._last_state = command.state
        return command

    def _update(self, snapshot, path, now, fork_passed, clear) -> MissionCommand:
        if self.stage is Stage.IDLE:
            return MissionCommand()
        if self.stage is Stage.DONE:
            return self._done(now)
        if self._dock_at_spot:
            self._dock_at_spot = False
            letter, track_id = self._spot
            role = Stage.PICKUP if letter == self._pickup else None
            self.dock = Dock(letter, track_id, role, now)

        stale = snapshot.age(now) > _STALE_S
        tracks = [] if stale else [t for t in snapshot.tracks if t.confirmed]

        self._consumed &= {t.id for t in snapshot.tracks}
        self._fix_position(tracks)
        self._follow_sign(snapshot, stale, now)
        request = self._follow_forks(fork_passed, now)
        decided = self._decide_detour(tracks, path, now)
        command = self._at_dock(now, clear) if self.dock is not None else self._update_approach(tracks, now, clear)
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
        self.decisions += 1
        self.last_decision = (sign.id, exit_number, wanted)
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

    def _update_approach(self, tracks: list[Track], now: float, clear: bool) -> MissionCommand:
        approach = self._approach
        if approach is not None:
            approach.observe(next((t for t in tracks if t.id == approach.id), None), now)
            if now - approach.last_seen > _LOST_S:
                logger.info("mission: %s #%d lost for %.1f s, approach abandoned",
                            approach.letter, approach.id, _LOST_S)
                self._approach = approach = None
            elif now - approach.committed_at > _MAX_APPROACH_S:
                logger.warning("mission: %s #%d %.0f s approaching, still %.2f m - stopping here",
                               approach.letter, approach.id, _MAX_APPROACH_S, approach.distance or -1.0)
                return self._arrive(approach, now, clear)

        if approach is None:
            candidates = [t for t in tracks if t.label == "delivery_sign" and t.variant == self.target
                          and t.id not in self._consumed and _within(t, _ACT_DISTANCE_M)]
            if candidates:
                nearest = min(candidates, key=lambda t: t.distance_m)
                self._approach = approach = _Approach(nearest, now)
                logger.info("mission: %s #%d at %.2f m, approaching for %s",
                            nearest.variant, nearest.id, nearest.distance_m, self.stage.value)

        if approach is not None and approach.distance <= _STOP_DISTANCE_M:
            return self._arrive(approach, now, clear)

        if approach is not None:
            return MissionCommand(scale=_ramp(approach.distance), turn_signal=TurnSignal.RIGHT,
                                  state=MissionState.SLOWING,
                                  reason=f"{self.stage.value} at {approach.letter} ahead")

        if self._leaving_until is not None:
            if now < self._leaving_until:
                return MissionCommand(turn_signal=TurnSignal.LEFT, state=MissionState.LEAVING,
                                      reason=f"leaving for {self.target}")
            self._leaving_until = None
        return MissionCommand(state=MissionState.DRIVING, reason=f"{self.stage.value} at {self.target}")

    def _arrive(self, approach: _Approach, now: float, clear: bool) -> MissionCommand:
        logger.info("mission: docked at %s (%.2f m) for %s", approach.letter, approach.distance or -1.0,
                    self.stage.value)
        self._consumed.add(approach.id)
        self._approach = None
        self.dock = Dock(approach.letter, approach.id, self.stage, now)
        return self._at_dock(now, clear)

    def _at_dock(self, now: float, clear: bool) -> MissionCommand:
        dock = self.dock
        self._follow_lid(dock, now)
        if dock.released_at is None:
            return self._waiting(dock, now)

        held = self._check_cargo(dock, now)
        if held is not None:
            return held
        if dock.role is Stage.DELIVERY:
            return self._deliver(now)
        if dock.role is Stage.PICKUP and self._pickup == self._delivery:
            # Loaded and unloaded at the same dock: nothing to drive.
            logger.info("mission: picked up at %s, delivering here", dock.letter)
            self.stage = Stage.DELIVERY
            self.dock = Dock(dock.letter, dock.track_id, Stage.DELIVERY, now)
            command = self._at_dock(now, clear)
            command.announce = Announcement(self._dock_key("docked", Stage.DELIVERY), dock.letter)
            return command

        if not self._quiet:
            # Still telling the people at the dock that it is leaving: the
            # wait for a clear way starts once they have heard it all.
            dock.clear_since = None
            return self._checking(dock, "announcing", "announcing the departure", None)
        announce = None
        if clear:
            if dock.clear_since is None:
                dock.clear_since = now
            if now - dock.clear_since >= _DEPART_CLEAR_S:
                return self._depart(now)
        else:
            dock.clear_since = None
            # From the frame after "released", which it would otherwise replace.
            if not dock.blocked_announced and self._last_state is MissionState.CHECKING:
                dock.blocked_announced = True
                announce = Announcement("blocked", dock.letter)
        waiting = "checking the way ahead" if clear else "waiting for the way ahead to clear"
        return self._checking(dock, "clearing" if clear else "blocked", waiting, announce)

    def _waiting(self, dock: Dock, now: float) -> MissionCommand:
        announce = None
        if dock.role is not None and dock.announced_at is not None and now - dock.announced_at >= _REMIND_S:
            announce = Announcement(self._dock_key("remind", dock.role), dock.letter)
        if dock.announced_at is None or announce is not None:
            dock.announced_at = now
        work = {Stage.PICKUP: "loading at", Stage.DELIVERY: "unloading at"}.get(dock.role, f"for {self.target} from")
        reason = f"{work} {dock.letter}, waiting for release"
        if dock.check_failed is not None and not dock.override:
            found = "nothing on board" if dock.check_failed == "empty" else "cargo still on board"
            reason += f" ({found}: release again to leave anyway)"
        return MissionCommand(scale=0.0, hold=True, turn_signal=TurnSignal.HAZARD, state=MissionState.DOCKED,
                              scene=DockScene.WAITING, announce=announce, reason=reason)

    def _checking(self, dock: Dock, check: str, waiting: str, announce: Optional[Announcement]) -> MissionCommand:
        return MissionCommand(scale=0.0, hold=True, turn_signal=TurnSignal.LEFT, state=MissionState.CHECKING,
                              scene=DockScene.CHECKING, announce=announce, check=check,
                              reason=f"leaving {dock.letter}, {waiting}")

    def _follow_lid(self, dock: Dock, now: float) -> None:
        """Lifted and shut again while the truck waits at a pickup or a
        delivery: the load is in (or out), and that is the release."""
        cargo = self._cargo
        if cargo is None or dock.role is None or dock.released_at is not None:
            return
        if not cargo.lid_closed:
            if not dock.lid_opened:
                logger.info("mission: compartment opened at %s", dock.letter)
            dock.lid_opened = True
        elif dock.lid_opened:
            logger.info("mission: compartment closed at %s", dock.letter)
            self.release(now)

    def _check_cargo(self, dock: Dock, now: float) -> Optional[MissionCommand]:
        """The truck's own checks once released: the lid shut, then the
        weight, judged once. A command while they hold the truck, None once
        they pass (or with no cargo node to check with)."""
        cargo = self._cargo
        if cargo is None:
            return None
        if not cargo.lid_closed:
            dock.clear_since = None
            dock.weighing_since = None
            announce = None
            if not dock.lid_announced:
                # Instead of "released": the truck is not about to move.
                dock.lid_announced = True
                announce = Announcement("close_lid", dock.letter)
            return self._checking(dock, "lid", "waiting for the lid to close", announce)
        if dock.role is None or dock.weighed or dock.override or cargo.fault:
            return None

        if dock.weighing_since is None:
            dock.weighing_since = now
        if not cargo.stable and now - dock.weighing_since < _SETTLE_S:
            dock.clear_since = None
            return self._checking(dock, "weighing", "weighing the cargo", None)

        empty = cargo.grams < _EMPTY_G
        failed = {Stage.PICKUP: "empty" if empty else None,
                  Stage.DELIVERY: None if empty else "still_loaded"}[dock.role]
        if failed is None:
            dock.weighed = True
            logger.info("mission: %d g on board at %s", cargo.grams, dock.letter)
            return None
        logger.warning("mission: %d g on board after %s at %s - waiting again", cargo.grams,
                       "loading" if dock.role is Stage.PICKUP else "unloading", dock.letter)
        dock.check_failed = failed
        dock.unrelease()
        dock.announced_at = now
        command = self._waiting(dock, now)
        command.announce = Announcement(failed, dock.letter)
        return command

    def _dock_key(self, what: str, role: Stage) -> str:
        """The dock's prompt: with the compartment it says to open and shut the lid."""
        return f"{what}_{role.value}" + ("_lid" if self._cargo is not None else "")

    def _depart(self, now: float) -> MissionCommand:
        dock = self.dock
        self.dock = None
        self._spot = None
        if dock.role is Stage.PICKUP:
            self.stage = Stage.DELIVERY
            cargo = self._cargo
            self.manifest_g = cargo.grams if cargo is not None and not cargo.fault else None
            logger.info("mission: picked up at %s, delivering to %s", dock.letter, self._delivery)
        else:
            logger.info("mission: left %s for %s", dock.letter, self.target)
        self._leaving_until = now + _LEAVING_S
        return MissionCommand(turn_signal=TurnSignal.LEFT, state=MissionState.LEAVING,
                              reason=f"leaving {dock.letter}")

    def _deliver(self, now: float) -> MissionCommand:
        dock = self.dock
        logger.info("mission: delivered at %s", dock.letter)
        self.dock = None
        self._spot = (dock.letter, dock.track_id)
        self._delivered_at = now
        self.stage = Stage.DONE
        return self._done(now)

    def _done(self, now: float) -> MissionCommand:
        letter = self._spot[0] if self._spot is not None else self._delivery
        delivered = self._delivered_at is not None and now - self._delivered_at < _DELIVERED_S
        return MissionCommand(scale=0.0, hold=True, turn_signal=TurnSignal.OFF, state=MissionState.DONE,
                              scene=DockScene.DELIVERED if delivered else DockScene.PARKED,
                              reason=f"delivered at {letter}" if self._delivered_at is not None
                              else f"parked at {letter}")

    def _announce_change(self, state: MissionState) -> Optional[Announcement]:
        if state is MissionState.DOCKED and self.dock.role is not None:
            return Announcement(self._dock_key("docked", self.dock.role), self.dock.letter)
        if state is MissionState.CHECKING:
            return Announcement("released", self.dock.letter)
        if state is MissionState.DONE and self._delivered_at is not None:
            return Announcement("delivered", self._spot[0])
        return None


def _within(track: Track, distance: float) -> bool:
    return track.distance_m is not None and track.distance_m <= distance


def _ramp(distance: Optional[float]) -> float:
    if distance is None:
        return 1.0
    span = _ACT_DISTANCE_M - _STOP_DISTANCE_M
    return max(0.0, min(1.0, (distance - _STOP_DISTANCE_M) / span))
