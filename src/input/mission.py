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

LETTER_EXIT = {"A": 1, "B": 2, "C": None}



def _exit_after(exit_number: int, entered: bool) -> int:
    return 2 if exit_number == 1 and not entered else 1


_EXIT_AFTER_LETTER = {"A": _exit_after(1, True), "B": _exit_after(2, True), "C": 1}

_ACT_DISTANCE_M = 3.0
_STOP_DISTANCE_M = 1.5
_LOST_S = 1.0
_MAX_APPROACH_S = 8.0
_STALE_S = 0.4
_SAME_SIGN_S = 2.0
_SAME_SIGN_M = 0.5
_DEPART_CLEAR_S = 2.0
_LEAVING_S = 2.0
_REMIND_S = 30.0
_DELIVERED_S = 3.0
_EMPTY_G = 20
_SETTLE_S = 3.0


class Stage(Enum):
    IDLE = "idle"
    PICKUP = "pickup"
    DELIVERY = "delivery"
    DONE = "done"


class MissionState(Enum):
    IDLE = "idle"
    DRIVING = "driving"
    SLOWING = "slowing"
    DOCKED = "docked"
    CHECKING = "checking"
    LEAVING = "leaving"
    DONE = "done"


@dataclass(frozen=True)
class Announcement:
    key: str
    letter: str


@dataclass
class MissionCommand:
    scale: float = 1.0
    hold: bool = False
    path: Optional[Path] = None
    turn_signal: Optional[TurnSignal] = None
    state: MissionState = MissionState.IDLE
    reason: str = ""
    scene: Optional[DockScene] = None
    announce: Optional[Announcement] = None
    check: Optional[str] = None


class Dock:
    def __init__(self, letter: str, track_id: Optional[int], role: Optional[Stage], now: float):
        self.letter = letter
        self.track_id = track_id
        self.role = role
        self.since = now
        self.released_at: Optional[float] = None
        self.clear_since: Optional[float] = None
        self.announced_at: Optional[float] = None
        self.blocked_announced = False
        self.lid_opened = False
        self.check_failed: Optional[str] = None
        self.override = False
        self.weighing_since: Optional[float] = None
        self.weighed = False
        self.lid_announced = False
        self.departure_announced = False

    def unrelease(self) -> None:
        self.released_at = None
        self.clear_since = None
        self.blocked_announced = False
        self.weighing_since = None
        self.weighed = False
        self.lid_announced = False
        self.departure_announced = False


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
    steps: list[str] = []
    next_exit = 1 if start is None else _EXIT_AFTER_LETTER[start]
    c_ahead = start is not None and LETTER_EXIT[start] is not None
    here = start
    for target, role in ((pickup, "pickup"), (delivery, "delivery")):
        if target == here:
            steps.append(f"{target}: {role}")
            continue
        here = target
        while True:
            if LETTER_EXIT[target] is None and c_ahead:
                steps.append(f"{target}: {role}")
                c_ahead = False
                break
            exit_number = next_exit
            enter = LETTER_EXIT[target] == exit_number
            steps.append(f"exit {exit_number}: {'detour' if enter else 'straight'}")
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
        self._spot: Optional[tuple[str, Optional[int]]] = None
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
        self.decisions = 0
        self.last_decision: Optional[tuple[int, int, Path]] = None
        self._approach: Optional[_Approach] = None
        self._consumed: set[int] = set()
        self._fixed: set[int] = set()
        self._decided: set[int] = set()
        self._sign_ids: set[int] = set()
        self._sign_distance = float("inf")
        self._sign_seen = float("-inf")
        self._forks_ahead = 0
        self._pending_exit: Optional[int] = None
        self._leaving_until: Optional[float] = None
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
        return self._spot is not None and (self.dock is None or self.dock.released_at is None)

    @property
    def parked_at(self) -> Optional[str]:
        return self._spot[0] if self.parked else None

    def configure(self, pickup: Optional[str], delivery: Optional[str]) -> None:
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
            self._dock_at_spot = True
            if self._spot[1] is not None:
                self._fixed.add(self._spot[1])
                self._consumed.add(self._spot[1])
        logger.info("mission: pickup %s, delivery %s - %s", self._pickup, self._delivery,
                    " -> ".join(self.route))

    def reset(self) -> None:
        self._approach = None
        self._leaving_until = None
        if self.dock is not None:
            self.dock.unrelease()

    def release(self, now: float) -> Optional[str]:
        if self.dock is None:
            return "not at a dock"
        if self.dock.released_at is not None:
            return "already released"
        self.dock.released_at = now
        self.dock.lid_opened = False
        if self.dock.check_failed is not None and not self.dock.override:
            self.dock.override = True
            logger.info("mission: released again at %s, leaving without the weight check", self.dock.letter)
        logger.info("mission: released at %s after %.0f s", self.dock.letter, now - self.dock.since)
        return None

    def hold_departure(self) -> Optional[str]:
        if self.dock is None or self.dock.released_at is None:
            return "not leaving a dock"
        self.dock.unrelease()
        logger.info("mission: departure from %s held", self.dock.letter)
        return None

    @property
    def unlock(self) -> bool:
        dock = self.dock
        return (dock is not None and dock.role is not None and dock.released_at is None
                and not dock.lid_opened)

    @property
    def scene(self) -> Optional[DockScene]:
        if self.dock is not None:
            return DockScene.WAITING
        if self.stage is Stage.DONE and self._spot is not None:
            return DockScene.PARKED
        return None

    def update(self, snapshot: SignSnapshot, path: Path, now: float,
               fork_passed: bool = False, clear: bool = True,
               cargo: Optional[CargoReading] = None, quiet: bool = True) -> MissionCommand:
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

    def _fix_position(self, tracks: list[Track]) -> None:
        for track in tracks:
            if (track.label != "delivery_sign" or track.variant not in _EXIT_AFTER_LETTER
                    or track.id in self._fixed or not _within(track, _ACT_DISTANCE_M)):
                continue
            self._fixed.add(track.id)
            self._forks_ahead = 0
            fixed = _EXIT_AFTER_LETTER[track.variant]
            if fixed != self.next_exit:
                logger.warning("mission: read %s at %.1f m - next exit %d -> %d",
                               track.variant, track.distance_m, self.next_exit, fixed)
                self.next_exit = fixed

    def _follow_sign(self, snapshot, stale, now) -> None:
        for track in snapshot.tracks:
            if track.id in self._sign_ids:
                self._sign_seen = now
                if track.distance_m is not None:
                    self._sign_distance = min(self._sign_distance, track.distance_m)
        if stale:
            self._sign_seen = now

    def _follow_forks(self, fork_passed, now) -> Optional[Path]:
        if fork_passed and self._forks_ahead:
            self._forks_ahead -= 1
            logger.info("mission: a mouth passed on the main road, %d still ahead", self._forks_ahead)
            if self._pending_exit is not None and self._forks_ahead == 0:
                logger.info("mission: exit %d -> detour (armed)", self._pending_exit)
                self._pending_exit = None
                return Path.DETOUR
        if self._pending_exit is not None and now - self._sign_seen > _LOST_S:
            logger.warning("mission: exit %d's sign passed before the mouth before it was seen - "
                           "straight on, next lap", self._pending_exit)
            self._pending_exit = None
            self._forks_ahead = 0
        return None

    def _decide_detour(self, tracks, path, now) -> Optional[Path]:
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
            logger.info("mission: picked up at %s, delivering here", dock.letter)
            self.stage = Stage.DELIVERY
            self.dock = Dock(dock.letter, dock.track_id, Stage.DELIVERY, now)
            command = self._at_dock(now, clear)
            command.announce = Announcement(self._dock_key("docked", Stage.DELIVERY), dock.letter)
            return command

        announce = None
        if not dock.departure_announced:
            dock.departure_announced = True
            announce = Announcement(self._dock_key("released"), dock.letter)
        if not self._quiet:
            dock.clear_since = None
            return self._checking(dock, "announcing", "announcing the departure", announce)
        if clear:
            if dock.clear_since is None:
                dock.clear_since = now
            if now - dock.clear_since >= _DEPART_CLEAR_S:
                return self._depart(now)
        else:
            dock.clear_since = None
            if not dock.blocked_announced and announce is None:
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
        cargo = self._cargo
        if cargo is None:
            return None
        if not cargo.lid_closed:
            dock.clear_since = None
            dock.weighing_since = None
            announce = None
            if not dock.lid_announced:
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

    def _dock_key(self, what: str, role: Optional[Stage] = None) -> str:
        return what + ("" if role is None else f"_{role.value}") + ("_lid" if self._cargo is not None else "")

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
