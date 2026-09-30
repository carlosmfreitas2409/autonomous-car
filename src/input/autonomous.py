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

_OBSTACLE_SLOW_CM = 150.0
_OBSTACLE_STOP_CM = 60.0

# Full-lock turning radius (m). The slow zone only counts echoes that lie on
# the car's path: a sensor pointing along the chassis sees the wall or cones
# on the outside of a bend well before the car would reach them, and on this
# track every tight right turn read 97-143 cm at full lock and cut the
# throttle by half for six seconds (day1_detour.csv 47-53 s, day0_normal.csv
# 85-88 s). The path turning at radius R has swung 90 degrees away after R
# metres, so an echo farther than R along the ray cannot be on it. The stop
# zone is unchanged: anything under 60 cm on any sensor still stops the car.
# PLACEHOLDER until measured with a chalk circle; the rule only ignores
# echoes beyond R, so overestimating R is the conservative error.
_R_LOCK_M = 0.6

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

# Leaving DETOUR for NORMAL waits for this many seconds of consecutive accepted
# readings with no hold running: an L1 pressed a moment before the blind merge
# must not hand a 0.2 s stop rule the blind stretch it was pressed for.
_PROFILE_RELEASE_S = 2.0


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

    def obstacle_scale(self, sonar, steering: float = 0.0) -> float:
        if sonar is None:
            return 1.0
        if sonar.blocked:
            return 0.0
        nearest = self._nearest_on_path(sonar, steering)
        if nearest is None or nearest >= _OBSTACLE_SLOW_CM:
            return 1.0
        if nearest <= _OBSTACLE_STOP_CM:
            return 0.0
        span = _OBSTACLE_SLOW_CM - _OBSTACLE_STOP_CM
        return (nearest - _OBSTACLE_STOP_CM) / span

    @staticmethod
    def _nearest_on_path(sonar, steering: float):
        """Nearest echo that the car's current path can reach.

        With the wheels at `steering` the path is an arc of radius about
        `_R_LOCK_M / |steering|`; beyond that distance along the ray the
        path has turned away. Echoes inside the stop zone always count.
        """
        distances = [d for d in getattr(sonar, "distances", ()) if d is not None]
        if not distances:
            distances = [] if sonar.nearest_cm is None else [sonar.nearest_cm]
        if not distances:
            return None
        reach_cm = float("inf") if abs(steering) < 1e-3 else 100.0 * _R_LOCK_M / abs(steering)
        on_path = [d for d in distances if d <= _OBSTACLE_STOP_CM or d <= reach_cm]
        return min(on_path) if on_path else None

    def step(
        self,
        lane_offset: Optional[int],
        lane_curvature: Optional[float],
        params: Config,
        dt: float,
        sonar=None,
        profile: ControlProfile = ControlProfile.NORMAL,
        provisional: bool = False,
    ) -> tuple[float, float]:
        obstacle = self.obstacle_scale(sonar, self._last_steering)
        self.stop_reason = "obstacle" if obstacle == 0.0 and sonar is not None else None

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
            return self._step_detour(lane_offset, lane_curvature, params, dt, obstacle, provisional)

        if lane_offset is None:
            # Lane lost: hold the last command for a short grace period (in case
            # it reappears), then stop. Don't reset the PID during the grace so
            # it resumes smoothly.
            self._lost_time += dt
            if self._lost_time <= _LANE_LOSS_TIMEOUT:
                self.holding = self._lost_time
                return self._last_steering, self._last_throttle * obstacle
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

        return steering, throttle * obstacle

    def _step_detour(self, lane_offset, lane_curvature, params, dt, obstacle, provisional) -> tuple[float, float]:
        holding = self._lost_time > 0.0
        # While a hold is running, a one-row bearing (or a frame with no
        # curvature when the scan has rows to fit one) is not a reading: the
        # line clipping a corner of the ROI on the way out of the blind merge
        # must not end the hold with a full-left kick.
        weak = provisional or (lane_curvature is None and params.scan_lines >= 2)
        if lane_offset is None or (holding and weak):
            self._lost_time += dt
            self._release_time = 0.0
            if self._lost_time <= _DETOUR_HOLD_S:
                self.holding = self._lost_time
                return self._last_steering, self._last_throttle * obstacle
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
        if holding or self._seed_next:
            # A seconds-old error must not feed the derivative.
            self._pid._prev_error = error
            self._seed_next = False
        steering = self._pid.update(error, dt)
        if dt > 0:
            max_step = _DETOUR_STEER_RATE * dt
            steering = self._last_steering + max(-max_step, min(max_step, steering - self._last_steering))

        throttle = self._throttle_for(steering, curvature, params)

        if self._releasing:
            self._release_time += dt
            if self._release_time >= _PROFILE_RELEASE_S:
                self._profile = ControlProfile.NORMAL
                self._releasing = False
                self._release_time = 0.0

        self._last_steering = steering
        self._last_throttle = throttle

        return steering, throttle * obstacle

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
