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
# the hold is full right lock, whatever the PID had reached: every recording
# that held 1.0 came out inside the lane (runs 1, 5, 6) and every one that
# held less came out wide (0.78 run 7, 0.73 run 8) or missed the fork (0.30
# run 2, -0.64 run 3). This also makes the detour independent of Kp, which
# the straights want lower than the detour would. A left excursion in the
# window is a correction on the way in, not the turn; with no right command
# in the window at all (a hallucinated loss on a straight) the last command
# is held as before.
_DETOUR_HOLD_LOOKBACK_S = 0.5
_DETOUR_TURN_SIGN = 1.0

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

        # Why the car is (or is about to be) held still, for the GUI and the log.
        self.stop_reason: Optional[str] = None
        # Seconds the last command has been held because the lane is lost.
        self.holding: float = 0.0

    def reset(self) -> None:
        self._pid.reset()
        self._lost_time = 0.0
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
    ) -> tuple[float, float]:
        """`obstacle` is the sonar guard's throttle scale; 0.0 means stopped."""
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
            return self._step_detour(lane_offset, lane_curvature, params, dt, obstacle, provisional, rows_found)

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

    def _step_detour(self, lane_offset, lane_curvature, params, dt, obstacle, provisional, rows_found) -> tuple[float, float]:
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
            or (rows_found is not None and rows_found <= rows // 2)
            or (lane_curvature is None and rows >= 2)
        )
        if lane_offset is None or weak:
            if not holding:
                self._latch_hold(params)
            self._lost_time += dt
            self._release_time = 0.0
            if self._lost_time <= _DETOUR_HOLD_S:
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

    def _latch_hold(self, params: Config) -> None:
        """Start a hold on the strongest recent command, not the last one."""
        recent = [s * _DETOUR_TURN_SIGN for t, s in self._recent if self._clock - t <= _DETOUR_HOLD_LOOKBACK_S]
        if not recent or max(recent) <= 0:
            return
        lock = _DETOUR_TURN_SIGN * self._pid.output_limits[1]
        self._last_steering = lock
        self._last_throttle = self._throttle_for(lock, 0.0, params)

    def _latch_hold(self, params: Config) -> None:
        """Start a hold on the strongest recent command, not the last one."""
        recent = [s * _DETOUR_TURN_SIGN for t, s in self._recent if self._clock - t <= _DETOUR_HOLD_LOOKBACK_S]
        if not recent or max(recent) <= 0:
            return
        peak = max(recent) * _DETOUR_TURN_SIGN
        if peak * _DETOUR_TURN_SIGN > self._last_steering * _DETOUR_TURN_SIGN:
            self._last_steering = peak
            self._last_throttle = self._throttle_for(peak, 0.0, params)

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
