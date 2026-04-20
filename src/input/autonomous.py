from typing import Optional

from config import Config


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

    def reset(self) -> None:
        self._pid.reset()

    def step(self, lane_offset: Optional[int], params: Config, dt: float) -> float:
        if lane_offset is None:
            self._pid.reset()
            return 0.0
        self._pid.kp = params.kp
        self._pid.ki = params.ki
        self._pid.kd = params.kd
        return self._pid.update(lane_offset, dt)
