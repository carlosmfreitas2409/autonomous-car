from enum import Enum
from typing import Optional

from input.autonomous import ControlProfile
from perception.line_tracking import LaneAnchor


class Path(Enum):
    NORMAL = "normal"
    STRAIGHT = "straight"
    DETOUR = "detour"


HOME_PATH = Path.STRAIGHT


def anchor_for(path: Path) -> LaneAnchor:
    if path is Path.STRAIGHT:
        return LaneAnchor.LEFT
    if path is Path.DETOUR:
        return LaneAnchor.RIGHT
    return LaneAnchor.AUTO


def profile_for(path: Path) -> ControlProfile:
    return ControlProfile.DETOUR if path is Path.DETOUR else ControlProfile.NORMAL


_RETURN_GAP_S = 0.3
_RETURN_STEADY_S = 2.0

_CARRY_HOLD_S = 20.0
_CARRY_HOLD_MIN_S = 0.3


class DetourReturn:
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
        if self._carry_time is not None:
            return ControlProfile.DETOUR
        return profile_for(path)

    def observe_hold(self, holding: float, dt: float, standing: bool = False) -> None:
        if standing:
            return
        if holding > 0:
            self._hold_run = holding
            if holding >= _CARRY_HOLD_MIN_S:
                self._after_merge = 0.0
        else:
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
        if standing:
            return None
        self._whole_lane = perc.lane_offset is not None and not perc.provisional and perc.both_rows >= 2
        self._lane_measured = (perc.lane_offset is not None and not perc.provisional
                               and (perc.both_rows > 0 or perc.band_rows >= 2))
        if path is not Path.DETOUR:
            if self._armed:
                self._leave()
            return None
        if not self._armed:
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
