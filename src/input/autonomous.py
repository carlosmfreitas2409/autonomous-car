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

    def reset(self) -> None:
        self._pid.reset()
        self._lost_time = 0.0
        self._last_steering = 0.0
        self._last_throttle = 0.0

    def step(self, lane_offset: Optional[int], lane_curvature: Optional[float], params: Config, dt: float) -> tuple[float, float]:
        if lane_offset is None:
            # Lane lost: hold the last command for a short grace period (in case
            # it reappears), then stop. Don't reset the PID during the grace so
            # it resumes smoothly.
            self._lost_time += dt
            if self._lost_time <= _LANE_LOSS_TIMEOUT:
                return self._last_steering, self._last_throttle
            self._pid.reset()
            self._last_steering = 0.0
            self._last_throttle = 0.0
            return 0.0, 0.0

        self._lost_time = 0.0
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
        return steering, throttle
