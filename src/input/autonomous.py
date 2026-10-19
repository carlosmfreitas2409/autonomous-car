from collections import deque
from enum import Enum
from typing import Optional

from config import Config


# Hold the last command this long when no lane is seen before stopping (seconds).
_LANE_LOSS_TIMEOUT = 0.2

# How many pixels of steering-error bias per unit of lane curvature (slope).
# Higher = the car turns earlier into a bend.
_LOOKAHEAD_GAIN = 60.0

# Curvature magnitude (slope) that demands a full slowdown to min_speed.
_CURVATURE_FULL = 0.5

# On the very sharpest bends the car may slow to this fraction of min_speed.
_SHARP_CURVE_FLOOR = 0.75

# DETOUR profile. A detour has stretches where no line is inside the ROI at
# all: ~1.2 s at the entrance and 4-5.5 s through the merge into the U-turn
# (detour_entry.mov, detour_exit.mov, driven by hand). The outer line is only
# visible at the very top of the frame there, and a far line does not say
# which way to steer; the driver's answer is to keep full lock until a line
# returns. So in DETOUR the car keeps its last steering and throttle for this
# long. The price: a car that has really left the track keeps circling for up
# to 8 s before it stops. The e-stop and the red `Stop:` label are the backstop.
# Frames steered on the line above the ROI (the tracker's far reading) are not
# blind and do not count: the 90-degree exit held 7.67 s on lap_c2, every frame
# after the first steered on that line, 0.3 s short of a stop.
_DETOUR_HOLD_S = 8.0

# DETOUR only: steering may change by at most this much per second (full
# range is 2.0, so lock to lock takes half a second, about what the actuator
# does anyway). Coming out of the blind stretch the first two-row frames on
# detour_exit.mov read -0.54, -0.27, -0.09 against a needed +1.0; unbounded,
# the derivative term on those swung the wheels lock to lock in one frame.
_DETOUR_STEER_RATE = 4.0

# When the lane is lost the DETOUR hold latches the largest steering commanded
# in this many seconds before the loss, not the last frame's. The readings
# just before a blind stretch are the flaky ones: at the entrance the peeling
# line leaves the frame row by row and the last two or three frames read one
# or two rows, near zero or the wrong sign, and the slew let the wheel decay
# from 0.8 to 0.1 before the hold latched it (day2/day3 recordings, 2026-09-29,
# at higher speed; the car passed the fork straight). The driver's answer is
# to keep turning at the rate they were turning. Every blind stretch a
# right-hand detour has (entrance, merge, U-turn) is a right turn, and the
# driver's answer to all of them is full lock until a line returns. So when
# the recent window shows the car was turning right into the blind stretch,
# the hold is a hard right turn (`_DETOUR_HOLD_STEER`, full lock until
# 2026-10-16), whatever the PID had reached: every recording
# that held 1.0 came out inside the lane (runs 1, 5, 6) and every one that
# held less came out wide (0.78 run 7, 0.73 run 8) or missed the fork (0.30
# run 2, -0.64 run 3). This also makes the detour independent of Kp, which
# the straights want lower than the detour would. A left excursion in the
# window is a correction on the way in, not the turn; with no right command
# in the window at all (a hallucinated loss on a straight) the last command
# is held as before.
_DETOUR_HOLD_LOOKBACK_S = 0.5
_DETOUR_TURN_SIGN = 1.0

# The steering a blind right-turn hold keeps. Full lock was the September
# answer (runs that held 0.30 or -0.64 missed the fork, 0.73-0.78 ran wide at
# the U-turn), but the exit now steers on the line above the ROI from the
# hold's second frame, and at the entrance full lock turns tighter than the
# detour's curve: on lap_e4 (2026-10-16) the car held it blind for 1.4 s and
# came out with the right tape beside the right wheels. The entrance has no
# line to steer on while blind (above the ROI it is the main road's line), so
# this value is the entry. A delay before the turn-in (0.15 and 0.25 s) moved
# the wheels 0.3 s later and changed nothing. Tuned on the track (2026-10-19,
# camera fixed, base 0.80 / min 0.67): 0.70 enters cleanly on every test, from
# about 0.75 up it cuts the right tape again. Lower if the entry cuts, higher if
# it runs wide to the island tip; re-tune if the camera or the speeds change.
_DETOUR_HOLD_STEER = 0.70

# What the ROI normally resolves on the main road: the rows found on the last
# this many NORMAL-profile frames with a lane (about 3 s). A DETOUR frame is
# weak below that, and never above half the scan lines. lap_d1 (2026-10-15):
# the GoPro was remounted, the ROI saw the line on 2 of 5 rows on every
# straight, every frame after R1 was "weak", and the car held the wheels
# straight through the fork. Straight drove that ROI without trouble.
_TYPICAL_ROWS_FRAMES = 90
_TYPICAL_ROWS_MIN_FRAMES = 30

# Leaving DETOUR for NORMAL waits for this many seconds of consecutive accepted
# readings with no hold running: an L1 pressed a moment before the blind merge
# must not hand a 0.2 s stop rule the blind stretch it was pressed for.
_PROFILE_RELEASE_S = 2.0


def _slowed(throttle: float, obstacle: float, params: Config) -> float:
    """Throttle scaled by the sonar guard, but never below the slowest the
    controller drives anywhere else (`min_speed` on the sharpest bend). The
    heavy car stalls under that, and stalling short of the stop distance left
    it standing in the slow zone without the stop, so without the hazards."""
    crawl = params.min_speed * _SHARP_CURVE_FLOOR
    return max(throttle * obstacle, min(throttle, crawl))


class ControlProfile(Enum):
    NORMAL = "normal"
    DETOUR = "detour"


class _PIDController:
    def __init__(self, kp: float, ki: float, kd: float, output_limits: tuple[float, float] = (-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        min_out, max_out = self.output_limits
        output = max(min_out, min(max_out, output))

        # Anti-windup: clamp integral when output is saturated
        if output == min_out or output == max_out:
            self._integral -= error * dt

        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


class Autonomous:
    def __init__(self):
        self._pid = _PIDController(kp=0.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0))
        self._lost_time = 0.0
        self._blind_time = 0.0
        self._last_steering = 0.0
        self._last_throttle = 0.0

        self._profile = ControlProfile.NORMAL
        self._release_time = 0.0
        self._releasing = False
        self._seed_next = False
        self._resume_seed = False
        # (time, steering) of recent accepted DETOUR readings, for the hold.
        self._recent: list[tuple[float, float]] = []
        self._clock = 0.0
        # Rows the ROI resolved on recent main-road (NORMAL) frames.
        self._main_rows: deque = deque(maxlen=_TYPICAL_ROWS_FRAMES)

        # Why the car is (or is about to be) held still, for the GUI and the log.
        self.stop_reason: Optional[str] = None
        # Seconds the last command has been held because the lane is lost.
        self.holding: float = 0.0

    def reset(self) -> None:
        self._pid.reset()
        self._lost_time = 0.0
        self._blind_time = 0.0
        self._last_steering = 0.0
        self._last_throttle = 0.0
        self._profile = ControlProfile.NORMAL
        self._release_time = 0.0
        self._releasing = False
        self._seed_next = False
        self._resume_seed = False
        self._recent = []
        self.stop_reason = None
        self.holding = 0.0

    @staticmethod
    def crawl_speed(params: Config) -> float:
        """The slowest throttle the car still moves at: the sharp-bend floor.

        Anything a policy ramps the throttle down to has to stay at or above
        this, or the car stalls short of where the policy wanted it to stop.
        """
        return params.min_speed * _SHARP_CURVE_FLOOR

    @property
    def profile(self) -> ControlProfile:
        return self._profile

    @property
    def profile_label(self) -> str:
        if self._profile is ControlProfile.NORMAL:
            return "normal"
        return "detour (releasing)" if self._releasing else "detour"

    def seed_derivative(self) -> None:
        """Make the next accepted reading seed the derivative instead of
        differencing against a stale error (used on a path change)."""
        self._seed_next = True

    def step(
        self,
        lane_offset: Optional[int],
        lane_curvature: Optional[float],
        params: Config,
        dt: float,
        obstacle: float = 1.0,
        profile: ControlProfile = ControlProfile.NORMAL,
        provisional: bool = False,
        rows_found: Optional[int] = None,
        far_offset: Optional[int] = None,
        far_curvature: Optional[float] = None,
    ) -> tuple[float, float]:
        """`obstacle` is the sonar guard's throttle scale; 0.0 means stopped.
        `far_offset`/`far_curvature` are the tracker's reading above a blind
        ROI; only a DETOUR hold steers on them."""
        if obstacle == 0.0:
            # Stopped for an obstacle: hold the wheels where they are. The
            # lane reading of a car standing still is noise, and steering on
            # it swung the wheels side to side while waiting. The lane-loss
            # hold and the profile release are frozen too (`holding` keeps
            # its value); the car is not moving.
            self.stop_reason = "obstacle"
            self._resume_seed = True
            return self._last_steering, 0.0
        self.stop_reason = None

        # Profile latch: DETOUR takes effect at once; NORMAL is deferred until
        # the lane has been steadily read for `_PROFILE_RELEASE_S` with no hold
        # running, so a running hold is never shortened by a path change.
        if profile is ControlProfile.DETOUR:
            self._profile = ControlProfile.DETOUR
            self._releasing = False
            self._release_time = 0.0
        elif self._profile is ControlProfile.DETOUR:
            self._releasing = True

        if self._profile is ControlProfile.DETOUR:
            return self._step_detour(lane_offset, lane_curvature, params, dt, obstacle, provisional, rows_found,
                                     far_offset, far_curvature)

        if lane_offset is None:
            # Lane lost: hold the last command for a short grace period (in case
            # it reappears), then stop. Don't reset the PID during the grace so
            # it resumes smoothly.
            self._lost_time += dt
            if self._lost_time <= _LANE_LOSS_TIMEOUT:
                self.holding = self._lost_time
                return self._last_steering, _slowed(self._last_throttle, obstacle, params)
            self._pid.reset()
            self._last_steering = 0.0
            self._last_throttle = 0.0
            self.stop_reason = "lane lost"
            self.holding = 0.0
            return 0.0, 0.0

        self._lost_time = 0.0
        self.holding = 0.0
        if rows_found is not None:
            self._main_rows.append(rows_found)
        self._pid.kp = params.kp
        self._pid.ki = params.ki
        self._pid.kd = params.kd

        # Anticipatory steering: bias the error toward where the lane is heading
        # so the car turns earlier into the bend instead of reacting late.
        curvature = lane_curvature if lane_curvature is not None else 0.0
        error = lane_offset + _LOOKAHEAD_GAIN * curvature
        if self._resume_seed:
            # The last error is from before the obstacle stop.
            self._pid._prev_error = error
            self._resume_seed = False
        steering = self._pid.update(error, dt)

        # Adaptive curve speed: slow for the larger of the current steering and
        # the upcoming curvature, so the car slows *before* the bend. The rule
        # only ever reduces speed within [floor, base_speed].
        curve_demand = abs(curvature) / _CURVATURE_FULL
        demand = max(abs(steering), curve_demand)
        if demand <= 1.0:
            throttle = params.base_speed - demand * (params.base_speed - params.min_speed)
        else:
            # Very sharp bend: dip below min_speed toward a bounded floor.
            extra = min(1.0, demand - 1.0)
            floor = params.min_speed * _SHARP_CURVE_FLOOR
            throttle = params.min_speed - extra * (params.min_speed - floor)

        self._last_steering = steering
        self._last_throttle = throttle

        return steering, _slowed(throttle, obstacle, params)

    def _step_detour(self, lane_offset, lane_curvature, params, dt, obstacle, provisional, rows_found,
                     far_offset=None, far_curvature=None) -> tuple[float, float]:
        self._clock += dt
        holding = self._lost_time > 0.0
        # A weak frame is not a reading in DETOUR: a one-row bearing, a
        # minority of the scan rows, or no curvature when the scan has rows to
        # fit one. On the way out of the blind merge the first thing the ROI
        # meets is a line clipping one corner of it (full-left readings); on
        # the way into a blind stretch the last frames are the peeling line's
        # last one or two rows (near-zero readings). Either would end or
        # weaken the hold the blind stretch needs.
        rows = max(1, params.scan_lines)
        weak = (
            provisional
            or (rows_found is not None and rows_found <= self._weak_rows(rows))
            or (lane_curvature is None and rows >= 2)
        )
        if lane_offset is None or weak:
            if not holding:
                self._latch_hold(params)
            elif far_offset is not None:
                self._steer_far(far_offset, far_curvature, params, dt)
            self._lost_time += dt
            self._blind_time = 0.0 if far_offset is not None else self._blind_time + dt
            self._release_time = 0.0
            if self._blind_time <= _DETOUR_HOLD_S:
                self.holding = self._lost_time
                return self._last_steering, _slowed(self._last_throttle, obstacle, params)
            self._pid.reset()
            self._last_steering = 0.0
            self._last_throttle = 0.0
            self.stop_reason = "lane lost"
            self.holding = 0.0
            return 0.0, 0.0

        self._lost_time = 0.0
        self._blind_time = 0.0
        self.holding = 0.0
        self._pid.kp = params.kp
        self._pid.ki = params.ki
        self._pid.kd = params.kd

        curvature = lane_curvature if lane_curvature is not None else 0.0
        error = lane_offset + _LOOKAHEAD_GAIN * curvature
        if holding or self._seed_next or self._resume_seed:
            # A seconds-old error must not feed the derivative.
            self._pid._prev_error = error
            self._seed_next = False
            self._resume_seed = False
        steering = self._pid.update(error, dt)
        if dt > 0:
            max_step = _DETOUR_STEER_RATE * dt
            steering = self._last_steering + max(-max_step, min(max_step, steering - self._last_steering))

        throttle = self._throttle_for(steering, curvature, params)

        self._recent.append((self._clock, steering))
        self._recent = [(t, s) for t, s in self._recent if self._clock - t <= _DETOUR_HOLD_LOOKBACK_S]

        if self._releasing:
            self._release_time += dt
            if self._release_time >= _PROFILE_RELEASE_S:
                self._profile = ControlProfile.NORMAL
                self._releasing = False
                self._release_time = 0.0

        self._last_steering = steering
        self._last_throttle = throttle

        return steering, _slowed(throttle, obstacle, params)

    def _weak_rows(self, scan_lines: int) -> int:
        """Most rows a DETOUR frame may resolve and still be weak: half the scan
        lines, or one less than the ROI typically resolves on the main road when
        that is lower (see `_TYPICAL_ROWS_FRAMES`)."""
        limit = scan_lines // 2
        if len(self._main_rows) >= _TYPICAL_ROWS_MIN_FRAMES:
            typical = sorted(self._main_rows)[len(self._main_rows) // 2]
            limit = min(limit, typical - 1)
        return limit

    def _latch_hold(self, params: Config) -> None:
        """Start a hold at `_DETOUR_HOLD_STEER` right when the car was turning
        right in the last `_DETOUR_HOLD_LOOKBACK_S`, whatever the last frame said."""
        recent = [s * _DETOUR_TURN_SIGN for t, s in self._recent if self._clock - t <= _DETOUR_HOLD_LOOKBACK_S]
        if not recent or max(recent) <= 0:
            return
        lock = _DETOUR_TURN_SIGN * _DETOUR_HOLD_STEER
        self._last_steering = lock
        self._last_throttle = self._throttle_for(lock, 0.0, params)

    def _steer_far(self, far_offset: int, far_curvature: Optional[float], params: Config, dt: float) -> None:
        """Steer a running hold on the line above the ROI instead of holding blind.

        The error is the ROI's (offset plus the curvature lookahead), proportional
        only: the reading is raw and a derivative over it would kick. Slew-limited
        like every DETOUR command. The hold's clock keeps running, so the 8 s stop
        and the detour return's merge detection are unchanged.
        """
        curvature = far_curvature if far_curvature is not None else 0.0
        lo, hi = self._pid.output_limits
        target = max(lo, min(hi, params.kp * (far_offset + _LOOKAHEAD_GAIN * curvature)))
        if dt > 0:
            max_step = _DETOUR_STEER_RATE * dt
            target = self._last_steering + max(-max_step, min(max_step, target - self._last_steering))
        self._last_steering = target
        self._last_throttle = self._throttle_for(target, 0.0, params)

    @staticmethod
    def _throttle_for(steering: float, curvature: float, params: Config) -> float:
        """The NORMAL curve-speed rule, unchanged: slow for the larger of the
        current steering and the upcoming curvature, within [floor, base_speed]."""
        curve_demand = abs(curvature) / _CURVATURE_FULL
        demand = max(abs(steering), curve_demand)
        if demand <= 1.0:
            return params.base_speed - demand * (params.base_speed - params.min_speed)
        extra = min(1.0, demand - 1.0)
        floor = Autonomous.crawl_speed(params)
        return params.min_speed - extra * (params.min_speed - floor)
