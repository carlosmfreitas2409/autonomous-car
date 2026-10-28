"""The operator's path choice at a fork, and what it means to perception and control.

The track is a clockwise loop whose detours all leave to the right. Two facts
follow: the main road's left line is continuous everywhere, and a detour's
right line is continuous from its entrance until past its merge. So the path
the operator picks maps straight onto which lane edge the tracker follows and
which control profile handles the blind stretches a detour has.

    Normal   - today's two-edge scan and today's controller, byte for byte.
    Straight - anchor on the left line: passes both forks without the
               midpoint drifting toward the peeling right line.
    Detour   - anchor on the right line and hold the last command through
               the blind entrance and the blind merge.

The choice is latched: nothing here returns to Normal by itself. E-stop and a
lane-lost timeout leave it alone. `HOME_PATH` is where the L1 button (and the
"(home)" label in the GUI) takes the car. Normal stays selectable forever.
"""

from enum import Enum
from typing import Optional

from input.autonomous import ControlProfile
from perception.line_tracking import LaneAnchor


class Path(Enum):
    NORMAL = "normal"
    STRAIGHT = "straight"
    DETOUR = "detour"


# Flipped to Straight on 2026-09-22: the Straight lap ran clean where the
# Normal lap oscillated, and the bias check passed on the day-0 recording.
HOME_PATH = Path.STRAIGHT


def anchor_for(path: Path) -> LaneAnchor:
    if path is Path.STRAIGHT:
        return LaneAnchor.LEFT
    if path is Path.DETOUR:
        return LaneAnchor.RIGHT
    return LaneAnchor.AUTO


def profile_for(path: Path) -> ControlProfile:
    return ControlProfile.DETOUR if path is Path.DETOUR else ControlProfile.NORMAL


# Automatic return from Detour. The merge cannot be detected reliably (the
# September attempt never fired on the real exit), but the entrance can: the
# right line peels away and for a moment the ROI sees a single line or nothing
# at all, then both detour lines are back and steady. Straight handles the
# exit and the U-turn as well as Detour does (live 2026-09-24), so Detour only
# has to last from the sign until the car is inside the detour.
_RETURN_GAP_S = 0.3        # entrance gap: this long without the lane measured
_RETURN_STEADY_S = 2.0     # then this long with both lines on two or more rows

# After the return (automatic, or L1 pressed inside the detour) the car is
# still inside the detour and the blind merge is ahead: Straight's 0.2 s grace
# would stop it there (1.4 s blind on the day-1 recording; day-4 stopped
# exactly so after an early L1). So the DETOUR control profile stays carried
# until a hold of at least `_CARRY_HOLD_MIN_S` has run and, after it, the
# lane has been measured (both lines, in the ROI or the band above it) for
# `_RETURN_STEADY_S`: the merge is followed by the U-turn on the outer line
# alone, which needs the same rules (day-7 went back to the 0.2 s grace
# mid-U-turn). Or this long has passed. No carry when that already happened
# while armed. Both lines on two ROI rows was the first rule; with the ROI at
# 0.60-0.85 that is 16-27 % of frames and the carry ran to its 20 s cap,
# through the S-curve after the 90-degree exit (lap_a5/a6, 2026-10-13).
_CARRY_HOLD_S = 20.0
_CARRY_HOLD_MIN_S = 0.3


class DetourReturn:
    """Watches the tracker while Detour is armed and says when to go home.

    `update` returns HOME_PATH on the frame the car is judged to be inside the
    detour, otherwise None. Arming again (R1) restarts the watch; L1 or the
    radio still override at any time.
    """

    def __init__(self):
        self._armed = False
        self._gap_time = 0.0
        self._gap_seen = False
        self._steady_time = 0.0
        self._carry_time: Optional[float] = None
        self._hold_run = 0.0
        self._hold_done = False
        self._inside_seen = False
        self._merge_seen = False
        self._after_merge = 0.0
        self._whole_lane = False
        self._lane_measured = False

    @property
    def stage(self) -> str:
        if not self._armed:
            return "" if self._carry_time is None else f"hold carried {self._carry_time:.0f}s"
        if not self._gap_seen:
            return "waiting for the entrance"
        return f"inside {self._steady_time:.1f}s"

    def profile_for(self, path: Path) -> ControlProfile:
        """The control profile: the path's own, or DETOUR while a merge hold
        is still owed after an automatic return."""
        if self._carry_time is not None:
            return ControlProfile.DETOUR
        return profile_for(path)

    def observe_hold(self, holding: float, dt: float, standing: bool = False) -> None:
        """Feed the controller's hold time every frame: a completed hold of
        `_CARRY_HOLD_MIN_S` or more is the merge, after which no carry is
        owed and a running carry is released. Nothing counts while the car
        is `standing` (see `update`)."""
        if standing:
            return
        if holding > 0:
            self._hold_run = holding
            # A blind stretch starts the count again; a few weak frames in the
            # S-curve after the exit (0.16-0.23 s on lap_a6) are not one.
            if holding >= _CARRY_HOLD_MIN_S:
                self._after_merge = 0.0
        else:
            # Only a hold that ends once the car has been inside the detour
            # is the merge; the entrance hold ends before the first whole
            # lane inside, and a weak frame on arming is not a hold at all.
            if self._hold_run >= _CARRY_HOLD_MIN_S and self._inside_seen:
                self._merge_seen = True
            self._hold_run = 0.0
        if self._carry_time is None:
            return
        self._carry_time += dt
        if self._merge_seen and self._lane_measured:
            self._after_merge += dt
        if (self._merge_seen and self._after_merge >= _RETURN_STEADY_S) or self._carry_time >= _CARRY_HOLD_S:
            self._carry_time = None
            self._hold_done = True

    def _leave(self):
        self._armed = False
        if not self._hold_done:
            self._carry_time = 0.0
            self._after_merge = 0.0

    def update(self, path: Path, perc, dt: float, standing: bool = False):
        """`standing`: the car is held still (an obstacle, a sign, a dock);
        `observe_hold` takes the same. Neither the gap and steady-lane timers
        here nor the carry there may count a car standing still: a pickup
        docked inside the detour ran the carry to its 20 s cap and the car met
        the blind exit on Normal's 0.2 s grace (lap_b2, 2026-10-24)."""
        if standing:
            return None
        self._whole_lane = perc.lane_offset is not None and not perc.provisional and perc.both_rows >= 2
        self._lane_measured = (perc.lane_offset is not None and not perc.provisional
                               and (perc.both_rows > 0 or perc.band_rows >= 2))
        if path is not Path.DETOUR:
            if self._armed:
                # L1 or the radio: home by hand, with the merge possibly ahead.
                self._leave()
            return None
        if not self._armed:
            # Armed (again): the explicit Detour owns the profile, and the
            # merge has not been crossed on this arming yet.
            self._armed = True
            self._gap_time = 0.0
            self._gap_seen = False
            self._steady_time = 0.0
            self._carry_time = None
            self._hold_run = 0.0
            self._hold_done = False
            self._inside_seen = False
            self._merge_seen = False
            self._after_merge = 0.0

        measured = perc.both_rows > 0 or perc.band_rows >= 2
        in_gap = perc.lane_offset is None or not measured

        if not self._gap_seen:
            self._gap_time = self._gap_time + dt if in_gap else 0.0
            if self._gap_time >= _RETURN_GAP_S:
                self._gap_seen = True
            return None

        whole_lane = self._whole_lane
        if whole_lane:
            self._inside_seen = True
        self._steady_time = self._steady_time + dt if whole_lane else 0.0
        if self._steady_time >= _RETURN_STEADY_S:
            self._leave()
            return HOME_PATH
        return None


class ForkWatch:
    """Says when the car, outside Detour, has driven past a detour's mouth.

    The same picture DetourReturn waits for, seen from the main road: the
    right line peels away and for `_RETURN_GAP_S` the lane is not measured,
    then the line beyond the island is back and the lane is measured again
    for as long. `update` returns True on that frame. Two exits can sit a few
    metres apart; the mission uses this to arm the second one's Detour only
    once the first one's mouth is behind the car.
    """

    def __init__(self):
        self._reset()

    def _reset(self) -> None:
        self._gap_time = 0.0
        self._gap_seen = False
        self._back_time = 0.0

    def update(self, path: Path, perc, dt: float) -> bool:
        if path is Path.DETOUR:
            self._reset()
            return False

        measured = perc.lane_offset is not None and (perc.both_rows > 0 or perc.band_rows >= 2)
        if not self._gap_seen:
            self._gap_time = 0.0 if measured else self._gap_time + dt
            self._gap_seen = self._gap_time >= _RETURN_GAP_S
            self._back_time = 0.0
            return False

        self._back_time = self._back_time + dt if measured else 0.0
        if self._back_time >= _RETURN_GAP_S:
            self._reset()
            return True
        return False
