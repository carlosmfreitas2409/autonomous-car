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

# After the return the car is still inside the detour and the blind merge is
# ahead: Straight's 0.2 s grace would stop it there (1.4 s blind on the day-1
# recording). So the DETOUR control profile stays latched until one hold has
# run and the lane came back, or this long has passed.
_CARRY_HOLD_S = 30.0


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
        self._carry_held = False

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

    def observe_hold(self, holding: float, dt: float) -> None:
        """Feed the controller's hold time every frame so the carried
        profile is released once the merge has been crossed."""
        if self._carry_time is None:
            return
        self._carry_time += dt
        if holding > 0:
            self._carry_held = True
        elif self._carry_held or self._carry_time >= _CARRY_HOLD_S:
            self._carry_time = None
            self._carry_held = False

    def update(self, path: Path, perc, dt: float):
        if path is not Path.DETOUR:
            self._armed = False
            return None
        if self._carry_time is not None:
            # Re-armed by hand while the carry was still running: the
            # explicit Detour owns the profile now.
            self._carry_time = None
            self._carry_held = False
        if not self._armed:
            self._armed = True
            self._gap_time = 0.0
            self._gap_seen = False
            self._steady_time = 0.0

        measured = perc.both_rows > 0 or perc.band_rows >= 2
        in_gap = perc.lane_offset is None or not measured

        if not self._gap_seen:
            self._gap_time = self._gap_time + dt if in_gap else 0.0
            if self._gap_time >= _RETURN_GAP_S:
                self._gap_seen = True
            return None

        whole_lane = perc.lane_offset is not None and not perc.provisional and perc.both_rows >= 2
        self._steady_time = self._steady_time + dt if whole_lane else 0.0
        if self._steady_time >= _RETURN_STEADY_S:
            self._armed = False
            self._carry_time = 0.0
            self._carry_held = False
            return HOME_PATH
        return None
