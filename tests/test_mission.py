"""The mission against a scripted lap: exit 1 (A inside), exit 2 (B inside,
reached only straight past exit 1, since detour 1 rejoins beyond it), C on the
main road, round again. The script follows the mission's own path
requests, so a test reads as the drive the car would make, including the ones
where a sign is missed or a detour is taken by mistake."""

import dataclasses
import itertools

import pytest

from cargo import CargoReading
from input.mission import (_ACT_DISTANCE_M as ACT, _DELIVERED_S, _DEPART_CLEAR_S, _EMPTY_G, _LEAVING_S,
                           _REMIND_S, _SETTLE_S, _STOP_DISTANCE_M as STOP, Mission, MissionState, Stage,
                           plan_route)
from lights import DockScene, TurnSignal
from path import Path
from perception.sign_tracker import Track
from perception.sign_worker import SignSnapshot
from perception.traffic_light import LightState


BASE = 100.0
DT = 0.1


def track(id, label, distance, variant=None, confirmed=True):
    return Track(id=id, label=label, bbox=(0, 0, 10, 10), confidence=0.8, distance_m=distance,
                 confirmed=confirmed, light_state=LightState.UNKNOWN, light_reason="", rejected=False,
                 variant=variant, variant_reason="", first_seen=0.0, last_seen=0.0)


class Drive:
    """The car on the scripted track: a clock, a path and a log of what it did.
    With `auto_release` someone at every dock releases the truck as soon as
    it docks; `clear` is what the sonar says about the way ahead, `cargo`
    what the cargo node does (None: not connected), `quiet` whether the
    truck's voice is silent."""

    def __init__(self, pickup, delivery, auto_release=True):
        self.mission = Mission()
        self.mission.configure(pickup, delivery)
        self.auto_release = auto_release
        self.clear = True
        self.cargo = None
        self.quiet = True
        self.t = 0.0
        self.path = Path.STRAIGHT
        self.log: list[str] = []
        self.commands = []
        self._ids = itertools.count(1)
        self._dock = None

    @property
    def now(self):
        return BASE + self.t

    def frame(self, *tracks, age=0.0, fork_passed=False):
        snapshot = SignSnapshot(tracks=list(tracks), timestamp=BASE + self.t - age,
                                camera_open=True, detector_available=True)
        command = self.mission.update(snapshot, self.path, self.now, fork_passed=fork_passed, clear=self.clear,
                                      cargo=self.cargo, quiet=self.quiet)
        if command.path is not None:
            self.path = command.path
        dock = self.mission.dock
        if dock is not None and dock is not self._dock and dock.role is not None:
            self.log.append(f"{dock.letter}: {dock.role.value}")
        self._dock = dock
        if self.auto_release and command.state is MissionState.DOCKED:
            self.mission.release(self.now)
        self.commands.append(command)
        self.t += DT
        return command

    def dock_at(self, letter, sign_id=None):
        """Drive up to `letter` until the car docks there."""
        sign_id = next(self._ids) if sign_id is None else sign_id
        distance = ACT - 0.1
        while self.mission.dock is None:
            command = self.frame(track(sign_id, "delivery_sign", distance, letter))
            distance -= 0.1 * max(command.scale, 0.3)
        return sign_id

    def gap(self, seconds=4.0):
        for _ in range(round(seconds / DT)):
            if self.done:
                return
            self.frame()

    def sign(self, label, variant=None, start=4.0, end=0.5):
        """One sign from `start` to `end` metres. The car only moves when the
        mission lets it, and never slower than a crawl."""
        sign_id = next(self._ids)
        distance = start
        for _ in range(2000):
            if distance <= end or self.done:
                break
            command = self.frame(track(sign_id, label, distance, variant))
            if not command.hold:
                distance -= 0.1 * max(command.scale, 0.3)
        return sign_id

    def exit(self, number, sign=True, forced=False, readable=True):
        if sign:
            self.sign("detour")
        else:
            self.gap(1.0)
        took = forced or self.path is Path.DETOUR
        self.log.append(f"exit {number}: {'detour' if took else 'straight'}")
        if self.path is not Path.DETOUR:
            # Outside Detour the lane camera sees the mouth go by (ForkWatch).
            self.frame(fork_passed=True)
        if took:
            # DetourReturn sends the path home once the car is inside.
            self.gap(1.0)
            self.path = Path.STRAIGHT
            letter = "A" if number == 1 else "B"
            self.sign("delivery_sign", letter if readable else None)
        self.gap(4.0)
        return took

    def main_road_c(self, readable=True):
        self.sign("delivery_sign", "C" if readable else None)
        self.gap(4.0)

    def laps(self, count=3, missed=(), forced=(), unreadable=(), phantom=(), start=None):
        """Drive `count` laps, from the start line or parked at the letter
        `start`. Faults are sets of (lap, place): place is an exit number, or
        "C". Detour 1 rejoins past exit 2's mouth, so exit 2 is only met
        straight past exit 1; every way round passes C."""
        lap, place = 0, 1 if start in (None, "C") else "C"
        while lap < count and not self.done:
            if (lap, place) in phantom:
                self.sign("detour")
                self.gap(4.0)
            if place == "C":
                self.main_road_c(readable=(lap, "C") not in unreadable)
                lap, place = lap + 1, 1
                continue
            took = self.exit(place, sign=(lap, place) not in missed, forced=(lap, place) in forced,
                             readable=(lap, place) not in unreadable)
            place = 2 if place == 1 and not took else "C"
        return self

    @property
    def done(self):
        return self.mission.stage is Stage.DONE


PAIRS = [(p, d) for p in "ABC" for d in "ABC" if p != d]


# --- the route -------------------------------------------------------------

@pytest.mark.parametrize("pickup,delivery,route", [
    ("A", "B", ["exit 1: detour", "A: pickup", "exit 1: straight", "exit 2: detour", "B: delivery"]),
    ("B", "A", ["exit 1: straight", "exit 2: detour", "B: pickup", "exit 1: detour", "A: delivery"]),
    ("C", "A", ["exit 1: straight", "exit 2: straight", "C: pickup", "exit 1: detour", "A: delivery"]),
    ("A", "C", ["exit 1: detour", "A: pickup", "C: delivery"]),
    ("B", "C", ["exit 1: straight", "exit 2: detour", "B: pickup", "C: delivery"]),
    ("C", "B", ["exit 1: straight", "exit 2: straight", "C: pickup", "exit 1: straight",
                "exit 2: detour", "B: delivery"]),
])
def test_plan_route_matches_the_track(pickup, delivery, route):
    assert plan_route(pickup, delivery) == route


@pytest.mark.parametrize("pickup,delivery", PAIRS)
def test_every_pair_drives_its_planned_route(pickup, delivery):
    drive = Drive(pickup, delivery).laps()
    assert drive.done
    assert drive.log == plan_route(pickup, delivery)


def test_same_letter_loads_and_unloads_at_one_dock():
    drive = Drive("B", "B").laps()
    assert drive.log == ["exit 1: straight", "exit 2: detour", "B: pickup", "B: delivery"]
    assert drive.log == plan_route("B", "B")
    assert drive.done


@pytest.mark.parametrize("pickup,delivery,start,route", [
    ("C", "A", "A", ["C: pickup", "exit 1: detour", "A: delivery"]),
    ("A", "C", "A", ["A: pickup", "C: delivery"]),
    ("A", "B", "C", ["exit 1: detour", "A: pickup", "exit 1: straight", "exit 2: detour", "B: delivery"]),
    ("B", "A", "B", ["B: pickup", "exit 1: detour", "A: delivery"]),
    ("A", "C", "B", ["exit 1: detour", "A: pickup", "C: delivery"]),
])
def test_plan_route_from_a_parked_dock(pickup, delivery, start, route):
    assert plan_route(pickup, delivery, start) == route


def park_at(letter):
    """A drive that has just delivered at `letter`."""
    drive = Drive("B" if letter == "A" else "A", letter).laps()
    assert drive.done and drive.mission.parked
    return drive


@pytest.mark.parametrize("start", "ABC")
@pytest.mark.parametrize("pickup,delivery", PAIRS + [("A", "A")])
def test_next_order_leaves_from_where_it_was_delivered(start, pickup, delivery):
    drive = park_at(start)
    drive.mission.configure(pickup, delivery)
    drive.log = []
    drive.laps(start=start)
    assert drive.done
    assert drive.log == plan_route(pickup, delivery, start)


# --- failures become a delay -----------------------------------------------

def test_phantom_sign_before_exit_1_costs_nothing():
    # A phantom detour sign puts the count one ahead, so exit 1's sign is taken
    # for exit 2's. That Detour waits for the mouth after the phantom: exit 1's,
    # so it is armed past exit 1 and the car takes exit 2 for B after all.
    drive = Drive("B", "C").laps(phantom={(0, 1)})
    assert drive.log == ["exit 1: straight", "exit 2: detour", "B: pickup", "C: delivery"]


def test_wrong_detour_read_as_a_is_corrected():
    # The count says exit 2 at exit 1's sign, so exit 1 is entered for B. Reading
    # A puts the count back on exit 1, and B comes on the next lap - the live
    # 2026-10-07 loop, where A used to say "exit 2 next" and it never ended.
    drive = Drive("B", "C")
    drive.mission.next_exit = 2
    drive.laps(forced={(0, 1)})
    assert drive.log == ["exit 1: detour", "exit 1: straight", "exit 2: detour", "B: pickup", "C: delivery"]


def test_after_a_the_next_sign_is_exit_1_again():
    # Live 2026-10-07, pickup A, delivery B: detour 1 rejoins past exit 2's
    # mouth, so the sign after A is exit 1's, and it must be passed straight.
    drive = Drive("A", "B")
    drive.exit(1)
    assert drive.mission.stage is Stage.DELIVERY
    assert drive.mission.next_exit == 1
    drive.main_road_c()
    assert drive.exit(1) is False
    assert drive.exit(2) is True
    assert drive.done


def test_missed_exit_1_sign_costs_one_wrong_detour():
    # Exit 2's sign is counted as exit 1 and entered for A; reading B inside
    # fixes the count and A is picked up on the next lap.
    drive = Drive("A", "B").laps(missed={(0, 1)})
    assert drive.log == ["exit 1: straight", "exit 2: detour", "exit 1: detour", "A: pickup",
                         "exit 1: straight", "exit 2: detour", "B: delivery"]


def test_detour_entered_by_mistake_costs_nothing():
    # The car wanders into exit 2 although the mission said straight: B is not
    # the target, so nothing stops, and reading B agrees with the count.
    drive = Drive("C", "A").laps(forced={(0, 2)})
    assert drive.log == ["exit 1: straight", "exit 2: detour", "C: pickup", "exit 1: detour", "A: delivery"]


def test_unread_letter_is_retried_next_lap():
    drive = Drive("A", "C").laps(unreadable={(0, 1)})
    assert drive.log == ["exit 1: detour", "exit 1: detour", "A: pickup", "C: delivery"]


def test_double_counted_sign_is_corrected_by_c():
    # A phantom sign after exit 2 would make exit 1 look like exit 2; C in
    # between resets the count.
    drive = Drive("C", "A").laps(phantom={(0, "C")})
    assert drive.log == plan_route("C", "A")


# --- what counts as a sign -------------------------------------------------

def test_unconfirmed_and_far_signs_are_ignored():
    drive = Drive("A", "B")
    for _ in range(30):
        drive.frame(track(1, "detour", 2.5, confirmed=False), track(2, "detour", ACT + 0.5))
        drive.frame(track(3, "delivery_sign", 2.1, "A", confirmed=False))
    assert drive.path is Path.STRAIGHT
    assert drive.mission.next_exit == 1
    assert all(c.scale == 1.0 and not c.hold for c in drive.commands)


def test_letter_without_a_committed_variant_is_ignored():
    drive = Drive("A", "B")
    command = drive.frame(track(1, "delivery_sign", STOP, None))
    assert command.scale == 1.0 and not command.hold


def test_flickering_detour_sign_is_decided_once():
    drive = Drive("B", "A")
    drive.frame(track(1, "detour", 2.8))
    assert drive.mission.next_exit == 2
    for _ in range(10):
        drive.frame()
    drive.frame(track(2, "detour", 2.0))   # same sign back under a new id
    assert drive.mission.next_exit == 2


def test_stale_snapshot_decides_nothing():
    drive = Drive("A", "B")
    command = drive.frame(track(1, "detour", 2.5), age=1.0)
    assert command.path is None
    assert drive.mission.next_exit == 1


def test_no_sign_is_decided_while_inside_a_detour():
    # Detour still armed (DetourReturn has not sent it home): a sign out here
    # waits, and is decided once the path is home if it is still in range.
    drive = Drive("B", "A")
    drive.path = Path.DETOUR
    command = drive.frame(track(1, "detour", 2.5))
    assert command.path is None and drive.mission.next_exit == 1
    drive.path = Path.STRAIGHT
    command = drive.frame(track(1, "detour", 2.4))
    assert command.path is None                # target B is not in exit 1: already straight
    assert drive.mission.next_exit == 2


def test_back_to_back_exits_with_both_signs_in_view():
    # record1_signs.mov: exit 2's sign is in view at 4.4 m while exit 1's is
    # still at 1.25 m, and in view without a break from there on.
    drive = Drive("B", "A")
    for step in range(60):                     # exit 1's sign alone, 2.5 m -> 1.3 m
        drive.frame(track(1, "detour", 2.5 - 0.02 * step))
    for step in range(25):                     # both in view
        drive.frame(track(1, "detour", 1.3 - 0.016 * step), track(2, "detour", 4.4 - 0.04 * step))
    assert drive.mission.next_exit == 2 and drive.path is Path.STRAIGHT
    for step in range(30):                     # exit 2's sign alone, into range
        drive.frame(track(2, "detour", 3.4 - 0.03 * step))
    # Decided, but exit 1's mouth is not behind the car yet: arming Detour
    # here would take exit 1.
    assert drive.mission.next_exit == 1
    assert drive.path is Path.STRAIGHT
    drive.frame(track(2, "detour", 2.4), fork_passed=True)
    assert drive.path is Path.DETOUR


def test_waiting_detour_is_dropped_when_its_sign_goes_by():
    # The mouth before it never showed (ForkWatch missed it): arming after the
    # sign has gone by would come after this exit's own mouth.
    drive = Drive("B", "A")
    drive.frame(track(1, "detour", 2.5))       # exit 1: straight
    drive.frame(track(1, "detour", 1.0))
    drive.frame(track(2, "detour", 2.9))       # exit 2: detour, waiting
    assert drive.path is Path.STRAIGHT and drive.mission.next_exit == 1
    drive.gap(2.0)
    drive.frame(fork_passed=True)              # exit 2's own mouth
    assert drive.path is Path.STRAIGHT
    assert drive.mission.next_exit == 1


def test_spaced_exits_arm_detour_at_the_sign():
    drive = Drive("B", "A")
    drive.frame(track(1, "detour", 2.5))       # exit 1: straight
    drive.gap(4.0)
    drive.frame(fork_passed=True)              # its mouth, long before the next sign
    drive.gap(4.0)
    drive.frame(track(2, "detour", 2.9))
    assert drive.path is Path.DETOUR


def test_lost_sign_found_again_under_a_new_id_is_not_a_new_exit():
    drive = Drive("B", "A")
    for step in range(10):
        drive.frame(track(1, "detour", 2.8 - 0.05 * step))
    drive.gap(1.5)                             # the tracker drops it...
    drive.frame(track(2, "detour", 2.0))       # ...and finds it again, closer
    assert drive.mission.next_exit == 2
    assert drive.path is Path.STRAIGHT


# --- the stop --------------------------------------------------------------

def test_pickup_docks_and_waits_as_long_as_it_takes():
    drive = Drive("C", "A", auto_release=False)
    drive.dock_at("C")
    docked = len(drive.commands) - 1
    drive.gap(100.0)
    approach, held = drive.commands[:docked], drive.commands[docked:]

    assert TurnSignal.RIGHT in [c.turn_signal for c in approach]
    assert all(c.scale < 1.0 for c in approach[-3:])
    assert all(c.hold and c.scale == 0.0 and c.turn_signal is TurnSignal.HAZARD
               and c.state is MissionState.DOCKED and c.scene is DockScene.WAITING for c in held)
    spoken = [c.announce.key for c in held if c.announce]
    assert spoken == ["docked_pickup"] + ["remind_pickup"] * int(100.0 // _REMIND_S)
    assert held[0].announce.letter == "C"
    assert drive.mission.stage is Stage.PICKUP


def test_release_signals_checks_the_way_then_leaves():
    drive = Drive("C", "A", auto_release=False)
    drive.dock_at("C")
    drive.gap(5.0)
    assert drive.mission.release(drive.now) is None
    released = len(drive.commands)
    drive.gap(_DEPART_CLEAR_S + _LEAVING_S + 1.0)
    after = drive.commands[released:]
    moving = next(i for i, c in enumerate(after) if not c.hold)

    checking = after[:moving]
    assert all(c.state is MissionState.CHECKING and c.turn_signal is TurnSignal.LEFT
               and c.scene is DockScene.CHECKING for c in checking)
    assert checking[0].announce.key == "released"
    assert len(checking) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    # Still signalling left on the first frames of motion, then clear.
    assert after[moving].state is MissionState.LEAVING and after[moving].turn_signal is TurnSignal.LEFT
    assert after[moving].scene is None
    assert drive.mission.stage is Stage.DELIVERY and drive.mission.dock is None
    assert after[-1].turn_signal is None and after[-1].state is MissionState.DRIVING


def test_departure_waits_for_the_voice_to_finish():
    drive = Drive("C", "A", auto_release=False)
    drive.dock_at("C")
    drive.mission.release(drive.now)
    assert drive.frame().announce.key == "released"
    drive.quiet = False                                     # "Liberado. Afaste-se..." playing
    drive.gap(5.0)
    assert all(c.hold and c.state is MissionState.CHECKING and "announcing" in c.reason
               for c in drive.commands[-50:])
    drive.quiet = True
    start = len(drive.commands)
    drive.gap(_DEPART_CLEAR_S + 1.0)
    moving = next(i for i, c in enumerate(drive.commands[start:]) if not c.hold)
    assert moving * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)


def test_someone_in_front_holds_the_departure():
    drive = Drive("C", "A", auto_release=False)
    drive.dock_at("C")
    drive.mission.release(drive.now)
    drive.clear = False
    start = len(drive.commands)
    drive.gap(10.0)
    blocked = drive.commands[start:]
    assert all(c.hold and c.state is MissionState.CHECKING for c in blocked)
    assert [c.announce.key for c in blocked if c.announce] == ["released", "blocked"]
    assert "clear" in blocked[-1].reason

    # A moment clear does not count: the 2 s start again after every echo.
    drive.clear = True
    drive.gap(_DEPART_CLEAR_S - 0.5)
    drive.clear = False
    drive.frame()
    drive.clear = True
    drive.gap(_DEPART_CLEAR_S - 0.3)
    assert drive.commands[-1].hold
    drive.gap(0.5)
    assert not drive.commands[-1].hold
    assert drive.mission.stage is Stage.DELIVERY


def test_wait_and_estop_need_a_new_release():
    drive = Drive("C", "A", auto_release=False)
    sign_id = drive.dock_at("C")
    dock = drive.mission.dock
    assert drive.mission.hold_departure() is not None       # nothing to hold yet
    drive.mission.release(drive.now)
    assert drive.frame().state is MissionState.CHECKING
    assert drive.mission.hold_departure() is None
    command = drive.frame()
    assert command.state is MissionState.DOCKED and command.announce.key == "docked_pickup"

    drive.mission.release(drive.now)
    drive.frame()
    drive.mission.reset()                                   # e-stop
    for _ in range(50):
        command = drive.frame(track(sign_id, "delivery_sign", STOP - 0.1, "C"))
    assert command.state is MissionState.DOCKED and command.hold
    assert drive.mission.dock is dock                       # the same dock, not a new approach
    assert drive.mission.stage is Stage.PICKUP


def test_release_away_from_a_dock_is_refused():
    assert Mission().release(BASE) is not None
    drive = Drive("C", "A", auto_release=False)
    drive.frame()
    assert drive.mission.release(drive.now) is not None     # driving
    command = drive.frame(track(1, "delivery_sign", 2.6, "C"))
    assert command.state is MissionState.SLOWING
    assert drive.mission.release(drive.now) is not None     # slowing
    assert drive.frame(track(1, "delivery_sign", 2.5, "C")).state is MissionState.SLOWING
    drive.dock_at("C", sign_id=1)
    assert drive.mission.release(drive.now) is None
    assert drive.mission.release(drive.now) is not None     # already released


def test_delivery_waits_then_parks_ready_for_the_next_order():
    drive = Drive("B", "C", auto_release=False)
    drive.mission.stage = Stage.DELIVERY
    drive.dock_at("C")
    docked = len(drive.commands) - 1
    drive.gap(60.0)
    assert all(c.state is MissionState.DOCKED and c.turn_signal is TurnSignal.HAZARD
               for c in drive.commands[docked:])
    assert drive.commands[docked].announce.key == "docked_delivery"
    assert not drive.mission.parked

    drive.mission.release(drive.now)
    start = len(drive.commands)
    for _ in range(300):
        drive.frame()
    after = drive.commands[start:]
    assert all(c.hold and c.scale == 0.0 and c.state is MissionState.DONE
               and c.turn_signal is TurnSignal.OFF for c in after)
    assert [c.announce.key for c in after if c.announce] == ["delivered"]
    delivered = sum(c.scene is DockScene.DELIVERED for c in after)
    assert delivered * DT == pytest.approx(_DELIVERED_S, abs=1.5 * DT)
    assert after[-1].scene is DockScene.PARKED
    assert drive.mission.parked and drive.mission.scene is DockScene.PARKED


def test_next_order_from_the_dock_waits_for_its_release():
    drive = park_at("C")
    drive.auto_release = False
    drive.mission.configure("A", "B")
    assert drive.mission.parked
    drive.gap(10.0)
    assert all(c.state is MissionState.DOCKED and c.announce is None for c in drive.commands[-100:])
    drive.mission.release(drive.now)
    assert not drive.mission.parked
    drive.gap(_DEPART_CLEAR_S + 0.5)
    assert not drive.commands[-1].hold
    assert drive.mission.stage is Stage.PICKUP and not drive.mission.parked


def test_half_made_order_while_parked_stays_parked():
    drive = park_at("A")
    drive.mission.configure("C", None)
    assert drive.mission.stage is Stage.DONE and drive.mission.parked
    command = drive.frame()
    assert command.hold and command.state is MissionState.DONE
    drive.mission.configure(None, None)                     # forgets the dock
    assert drive.mission.stage is Stage.IDLE and not drive.mission.parked
    assert not drive.frame().hold


def test_lost_letter_abandons_the_approach():
    drive = Drive("C", "A")
    command = drive.frame(track(1, "delivery_sign", 2.6, "C"))
    assert command.state is MissionState.SLOWING
    for _ in range(12):
        command = drive.frame()
    assert command.scale == 1.0 and not command.hold
    assert drive.mission.stage is Stage.PICKUP


# --- cargo compartment -----------------------------------------------------

LOADED = 300


def cargo(grams=0, lid=True, stable=True, fault=False):
    return CargoReading(grams=grams, stable=stable, lid_closed=lid, unlocked=False, fault=fault)


def lid_cycle(drive, grams):
    """Someone lifts the lid, loads or unloads to `grams` and shuts it: the
    frame it shuts on."""
    drive.cargo = dataclasses.replace(drive.cargo, grams=grams, lid_closed=False)
    drive.frame()
    drive.cargo = dataclasses.replace(drive.cargo, lid_closed=True)
    return drive.frame()


def leave(drive, seconds=10.0):
    """Frames until the car moves off (or `seconds` pass): those it held."""
    held = []
    for _ in range(round(seconds / DT)):
        command = drive.frame()
        if not command.hold:
            break
        held.append(command)
    return held


def test_compartment_unlocks_only_while_waiting_at_a_dock():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo()
    assert drive.frame().state is MissionState.DRIVING and not drive.mission.unlock
    assert drive.frame(track(1, "delivery_sign", 2.6, "C")).state is MissionState.SLOWING
    assert not drive.mission.unlock
    drive.dock_at("C", sign_id=1)
    drive.gap(5.0)
    assert drive.commands[-1].state is MissionState.DOCKED and drive.mission.unlock

    # Lifted: the lock lets go, the latch catches as the lid shuts.
    drive.cargo = cargo(LOADED, lid=False)
    drive.gap(5.0)
    assert drive.commands[-1].state is MissionState.DOCKED and not drive.mission.unlock
    drive.cargo = cargo(LOADED)
    assert drive.frame().state is MissionState.CHECKING and not drive.mission.unlock
    while drive.mission.stage is Stage.PICKUP:
        drive.frame()
        assert not drive.mission.unlock
    for _ in range(100):
        drive.frame()
        assert not drive.mission.unlock


def test_shutting_the_lid_releases_and_records_the_manifest():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo()
    drive.dock_at("C")
    assert drive.commands[-1].announce.key == "docked_pickup_lid"
    drive.gap(_REMIND_S + 1.0)
    # A lid never lifted releases nothing.
    assert all(c.state is MissionState.DOCKED for c in drive.commands[-100:])
    assert [c.announce.key for c in drive.commands[-100:] if c.announce] == ["remind_pickup_lid"]

    command = lid_cycle(drive, LOADED)
    assert command.state is MissionState.CHECKING and command.announce.key == "released"
    held = leave(drive)
    assert len(held) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    assert drive.mission.stage is Stage.DELIVERY and drive.mission.manifest_g == LOADED


def test_released_with_the_lid_open_waits_for_it():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo(LOADED, lid=False)
    drive.dock_at("C")
    drive.mission.release(drive.now)
    held = leave(drive, 10.0)
    assert len(held) == 100
    assert all(c.state is MissionState.CHECKING and "lid" in c.reason for c in held)
    assert [c.announce.key for c in held if c.announce] == ["close_lid"]

    drive.cargo = cargo(LOADED)
    held = leave(drive)
    assert len(held) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    assert drive.mission.stage is Stage.DELIVERY


def test_empty_pickup_waits_again_and_a_second_release_overrides():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo()
    drive.dock_at("C")
    command = lid_cycle(drive, _EMPTY_G - 1)
    assert command.state is MissionState.DOCKED and command.announce.key == "empty"
    assert "release again" in command.reason and drive.mission.unlock
    drive.gap(5.0)
    assert drive.commands[-1].state is MissionState.DOCKED

    assert drive.mission.release(drive.now) is None
    held = leave(drive)
    assert all(c.state is MissionState.CHECKING for c in held)
    assert len(held) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    assert drive.mission.stage is Stage.DELIVERY and drive.mission.manifest_g == _EMPTY_G - 1


@pytest.mark.parametrize("on_board, outcome", [(0, "delivered"), (LOADED, "still_loaded")])
def test_delivery_leaves_nothing_on_board(on_board, outcome):
    drive = Drive("B", "C", auto_release=False)
    drive.mission.stage = Stage.DELIVERY
    drive.cargo = cargo(LOADED)
    drive.dock_at("C")
    assert drive.commands[-1].announce.key == "docked_delivery_lid"
    command = lid_cycle(drive, on_board)
    assert command.announce.key == outcome
    if outcome == "still_loaded":
        assert command.state is MissionState.DOCKED and drive.mission.unlock
        assert lid_cycle(drive, 0).state is MissionState.DONE
    assert drive.done and drive.mission.parked


def test_unsettled_weight_is_judged_after_a_while():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo(LOADED, stable=False)
    drive.dock_at("C")
    drive.mission.release(drive.now)
    held = leave(drive)
    weighing = [c for c in held if "weighing" in c.reason]
    assert len(weighing) * DT == pytest.approx(_SETTLE_S, abs=1.5 * DT)
    assert len(held) * DT == pytest.approx(_SETTLE_S + _DEPART_CLEAR_S, abs=2.5 * DT)
    assert drive.mission.stage is Stage.DELIVERY


def test_scale_fault_skips_the_weight():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo(0, fault=True)
    drive.dock_at("C")
    lid_cycle(drive, 0)
    held = leave(drive)
    assert len(held) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    assert drive.mission.stage is Stage.DELIVERY and drive.mission.manifest_g is None


def test_cargo_node_lost_at_the_dock_leaves_as_without_it():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo(0, lid=False)
    drive.dock_at("C")
    drive.mission.release(drive.now)
    assert leave(drive, 3.0)[-1].state is MissionState.CHECKING
    drive.cargo = None
    held = leave(drive)
    assert len(held) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    assert drive.mission.stage is Stage.DELIVERY and drive.mission.manifest_g is None


def test_wait_reopens_the_compartment():
    drive = Drive("C", "A", auto_release=False)
    drive.cargo = cargo()
    drive.dock_at("C")
    lid_cycle(drive, LOADED)
    assert drive.mission.hold_departure() is None
    assert drive.frame().state is MissionState.DOCKED and drive.mission.unlock
    drive.gap(3.0)
    assert drive.commands[-1].state is MissionState.DOCKED
    assert lid_cycle(drive, LOADED).state is MissionState.CHECKING


def test_leaving_a_parked_dock_checks_the_lid_but_never_unlocks():
    drive = park_at("C")
    drive.auto_release = False
    drive.cargo = cargo(0, lid=False)
    drive.mission.configure("A", "B")
    drive.gap(3.0)
    assert drive.commands[-1].state is MissionState.DOCKED and not drive.mission.unlock
    drive.mission.release(drive.now)
    assert leave(drive, 3.0)[-1].reason.endswith("waiting for the lid to close")
    drive.cargo = cargo(0)
    held = leave(drive)
    assert len(held) * DT == pytest.approx(_DEPART_CLEAR_S, abs=1.5 * DT)
    assert drive.mission.stage is Stage.PICKUP


# --- inactive --------------------------------------------------------------

def test_letters_picked_one_at_a_time_start_the_run():
    mission = Mission()
    mission.configure("A", None)
    assert mission.stage is Stage.IDLE and mission.pickup == "A"
    mission.configure(mission.pickup, "B")
    assert mission.stage is Stage.PICKUP
    assert (mission.pickup, mission.delivery) == ("A", "B")


@pytest.mark.parametrize("letters", [(None, None), ("A", None), (None, "B")])
def test_idle_mission_never_acts(letters):
    drive = Drive(*letters)
    for distance in (2.9, 2.5, 2.0, 1.5):
        drive.frame(track(1, "detour", distance), track(2, "delivery_sign", distance, "A"))
    assert drive.mission.stage is Stage.IDLE
    assert all(c.path is None and c.scale == 1.0 and not c.hold and c.turn_signal is None
               for c in drive.commands)
