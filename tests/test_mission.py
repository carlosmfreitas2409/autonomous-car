"""The mission against a scripted lap: exit 1 (A inside), exit 2 (B inside),
C on the main road, round again. The script follows the mission's own path
requests, so a test reads as the drive the car would make, including the ones
where a sign is missed or a detour is taken by mistake."""

import itertools

import pytest

from input.mission import (_ACT_DISTANCE_M as ACT, _DWELL_S, _LEAVING_S, _SETTLE_S,
                           _SIGNAL_BEFORE_LEAVING_S, _STOP_DISTANCE_M as STOP,
                           Mission, MissionState, Stage, plan_route)
from lights import TurnSignal
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
    """The car on the scripted track: a clock, a path and a log of what it did."""

    def __init__(self, pickup, delivery):
        self.mission = Mission()
        self.mission.configure(pickup, delivery)
        self.t = 0.0
        self.path = Path.STRAIGHT
        self.log: list[str] = []
        self.commands = []
        self._ids = itertools.count(1)

    def frame(self, *tracks, age=0.0):
        snapshot = SignSnapshot(tracks=list(tracks), timestamp=BASE + self.t - age,
                                camera_open=True, detector_available=True)
        command = self.mission.update(snapshot, self.path, BASE + self.t)
        if command.path is not None:
            self.path = command.path
        self.commands.append(command)
        self.t += DT
        return command

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
        held = False
        for _ in range(2000):
            if distance <= end or self.done:
                break
            command = self.frame(track(sign_id, label, distance, variant))
            if command.hold and not held:
                role = "delivery" if self.mission.stage is Stage.DONE else "pickup"
                self.log.append(f"{variant}: {role}")
            held = command.hold
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
        if took:
            # DetourReturn sends the path home once the car is inside.
            self.gap(1.0)
            self.path = Path.STRAIGHT
            letter = "A" if number == 1 else "B"
            self.sign("delivery_sign", letter if readable else None)
        self.gap(4.0)

    def main_road_c(self, readable=True):
        self.sign("delivery_sign", "C" if readable else None)
        self.gap(4.0)

    def laps(self, count=3, missed=(), forced=(), unreadable=(), phantom=()):
        """Drive `count` laps. Faults are sets of (lap, place): place is an
        exit number, or "C"."""
        for lap in range(count):
            for place in (1, 2, "C"):
                if self.done:
                    return self
                if (lap, place) in phantom:
                    self.sign("detour")
                    self.gap(4.0)
                if place == "C":
                    self.main_road_c(readable=(lap, "C") not in unreadable)
                else:
                    self.exit(place, sign=(lap, place) not in missed, forced=(lap, place) in forced,
                              readable=(lap, place) not in unreadable)
        return self

    @property
    def done(self):
        return self.mission.stage is Stage.DONE


PAIRS = [(p, d) for p in "ABC" for d in "ABC" if p != d]


# --- the route -------------------------------------------------------------

@pytest.mark.parametrize("pickup,delivery,route", [
    ("A", "B", ["exit 1: detour", "A: pickup", "exit 2: detour", "B: delivery"]),
    ("B", "A", ["exit 1: straight", "exit 2: detour", "B: pickup", "exit 1: detour", "A: delivery"]),
    ("C", "A", ["exit 1: straight", "exit 2: straight", "C: pickup", "exit 1: detour", "A: delivery"]),
    ("A", "C", ["exit 1: detour", "A: pickup", "exit 2: straight", "C: delivery"]),
    ("B", "C", ["exit 1: straight", "exit 2: detour", "B: pickup", "C: delivery"]),
    ("C", "B", ["exit 1: straight", "exit 2: straight", "C: pickup", "exit 1: straight",
                "exit 2: detour", "B: delivery"]),
])
def test_plan_route_matches_the_task(pickup, delivery, route):
    assert plan_route(pickup, delivery) == route


@pytest.mark.parametrize("pickup,delivery", PAIRS)
def test_every_pair_drives_its_planned_route(pickup, delivery):
    drive = Drive(pickup, delivery).laps()
    assert drive.done
    assert drive.log == plan_route(pickup, delivery)


def test_same_letter_stops_there_for_good():
    drive = Drive("B", "B").laps()
    assert drive.log == ["exit 1: straight", "exit 2: detour", "B: delivery"]
    assert drive.done


# --- failures become a delay -----------------------------------------------

def test_wrong_detour_read_as_a_is_corrected():
    # A phantom detour sign before exit 1 puts the count one ahead, so exit 1
    # is taken for exit 2 and entered for B. Reading A fixes the count.
    drive = Drive("B", "C").laps(phantom={(0, 1)})
    assert drive.log == ["exit 1: detour", "exit 2: detour", "B: pickup", "C: delivery"]


def test_missed_exit_1_sign_costs_one_wrong_detour():
    # Exit 2's sign is counted as exit 1 and entered for A; reading B inside
    # fixes the count and A is picked up on the next lap.
    drive = Drive("A", "B").laps(missed={(0, 1)})
    assert drive.log == ["exit 1: straight", "exit 2: detour", "exit 1: detour", "A: pickup",
                         "exit 2: detour", "B: delivery"]


def test_detour_entered_by_mistake_costs_nothing():
    # The car wanders into exit 2 although the mission said straight: B is not
    # the target, so nothing stops, and reading B agrees with the count.
    drive = Drive("C", "A").laps(forced={(0, 2)})
    assert drive.log == ["exit 1: straight", "exit 2: detour", "C: pickup", "exit 1: detour", "A: delivery"]


def test_unread_letter_is_retried_next_lap():
    drive = Drive("A", "C").laps(unreadable={(0, 1)})
    assert drive.log == ["exit 1: detour", "exit 2: straight", "exit 1: detour", "A: pickup",
                         "exit 2: straight", "C: delivery"]


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


def test_detour_request_is_one_shot_and_skipped_when_already_set():
    drive = Drive("A", "B")
    drive.path = Path.DETOUR               # the operator already pressed R1
    command = drive.frame(track(1, "detour", 2.5))
    assert command.path is None
    assert drive.mission.next_exit == 2


# --- the stop --------------------------------------------------------------

def test_pickup_signals_ramp_dwell_and_release():
    drive = Drive("C", "A")
    drive.sign("delivery_sign", "C")
    signals = [c.turn_signal for c in drive.commands]
    states = [c.state for c in drive.commands]
    first_hold = next(i for i, c in enumerate(drive.commands) if c.hold)
    last_hold = max(i for i, c in enumerate(drive.commands) if c.hold)

    held = signals[first_hold:last_hold + 1]
    hazard = held.count(TurnSignal.HAZARD)

    assert TurnSignal.RIGHT in signals[:first_hold]
    assert all(c.scale < 1.0 for c in drive.commands[first_hold - 3:first_hold])
    # Hazards while stopped, then the left signal while still stopped: signal first, then go.
    assert held == [TurnSignal.HAZARD] * hazard + [TurnSignal.LEFT] * (len(held) - hazard)
    assert hazard * DT == pytest.approx(_SETTLE_S + _DWELL_S, abs=1.5 * DT)
    assert (len(held) - hazard) * DT == pytest.approx(_SIGNAL_BEFORE_LEAVING_S, abs=1.5 * DT)
    assert states[last_hold] is MissionState.LEAVING
    # Still signalling left on the first frames of motion, then clear.
    assert signals[last_hold + 1] is TurnSignal.LEFT
    assert not drive.commands[last_hold + 1].hold
    assert drive.mission.stage is Stage.DELIVERY

    drive.gap(_LEAVING_S + 1.0)
    assert drive.commands[-1].turn_signal is None


def test_delivery_holds_for_good_with_hazards():
    drive = Drive("B", "C")
    drive.mission.stage = Stage.DELIVERY
    drive.sign("delivery_sign", "C")
    drive.gap(20.0)
    for _ in range(50):
        command = drive.frame()
    assert command.hold and command.scale == 0.0
    assert command.turn_signal is TurnSignal.HAZARD
    assert command.state is MissionState.DONE


def test_estop_during_dwell_repeats_the_stop():
    drive = Drive("C", "A")
    for _ in range(12):
        command = drive.frame(track(1, "delivery_sign", STOP - 0.1, "C"))
    assert command.hold
    drive.mission.reset()
    command = drive.frame(track(1, "delivery_sign", STOP - 0.1, "C"))
    assert command.hold and command.state is MissionState.STOPPED
    assert drive.mission.stage is Stage.PICKUP


def test_lost_letter_abandons_the_approach():
    drive = Drive("C", "A")
    command = drive.frame(track(1, "delivery_sign", 2.6, "C"))
    assert command.state is MissionState.SLOWING
    for _ in range(12):
        command = drive.frame()
    assert command.scale == 1.0 and not command.hold
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
