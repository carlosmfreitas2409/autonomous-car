from collections import deque
from enum import Enum
from typing import Optional

from config import Config


_LANE_LOSS_TIMEOUT = 0.2

_LOOKAHEAD_GAIN = 60.0

_CURVATURE_FULL = 0.5

_SHARP_CURVE_FLOOR = 0.75

_DETOUR_HOLD_S = 8.0

_DETOUR_STEER_RATE = 4.0

_DETOUR_HOLD_LOOKBACK_S = 0.5
_DETOUR_TURN_SIGN = 1.0

_DETOUR_HOLD_STEER = 0.70

_TYPICAL_ROWS_FRAMES = 90
_TYPICAL_ROWS_MIN_FRAMES = 30

_PROFILE_RELEASE_S = 2.0


def _slowed(throttle: float, obstacle: float, params: Config) -> float:
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
        self._recent: list[tuple[float, float]] = []
        self._clock = 0.0
        self._main_rows: deque = deque(maxlen=_TYPICAL_ROWS_FRAMES)

        self.stop_reason: Optional[str] = None
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
        if obstacle == 0.0:
            self.stop_reason = "obstacle"
            self._resume_seed = True
            return self._last_steering, 0.0
        self.stop_reason = None

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

        curvature = lane_curvature if lane_curvature is not None else 0.0
        error = lane_offset + _LOOKAHEAD_GAIN * curvature
        if self._resume_seed:
            self._pid._prev_error = error
            self._resume_seed = False
        steering = self._pid.update(error, dt)

        curve_demand = abs(curvature) / _CURVATURE_FULL
        demand = max(abs(steering), curve_demand)
        if demand <= 1.0:
            throttle = params.base_speed - demand * (params.base_speed - params.min_speed)
        else:
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
        limit = scan_lines // 2
        if len(self._main_rows) >= _TYPICAL_ROWS_MIN_FRAMES:
            typical = sorted(self._main_rows)[len(self._main_rows) // 2]
            limit = min(limit, typical - 1)
        return limit

    def _latch_hold(self, params: Config) -> None:
        recent = [s * _DETOUR_TURN_SIGN for t, s in self._recent if self._clock - t <= _DETOUR_HOLD_LOOKBACK_S]
        if not recent or max(recent) <= 0:
            return
        lock = _DETOUR_TURN_SIGN * _DETOUR_HOLD_STEER
        self._last_steering = lock
        self._last_throttle = self._throttle_for(lock, 0.0, params)

    def _steer_far(self, far_offset: int, far_curvature: Optional[float], params: Config, dt: float) -> None:
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
        curve_demand = abs(curvature) / _CURVATURE_FULL
        demand = max(abs(steering), curve_demand)
        if demand <= 1.0:
            return params.base_speed - demand * (params.base_speed - params.min_speed)
        extra = min(1.0, demand - 1.0)
        floor = Autonomous.crawl_speed(params)
        return params.min_speed - extra * (params.min_speed - floor)
