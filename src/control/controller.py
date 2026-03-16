"""Main control orchestrator."""

from __future__ import annotations

from src.config import DEADZONE_THRESHOLD, THROTTLE_RAMP_RATE, STEERING_RAMP_RATE
from src.control.deadzone import apply_deadzone
from src.control.motor_mixer import mix
from src.control.smoother import Smoother
from src.models.commands import ControlCommand, MotorCommand

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.safety.emergency_stop import EStopManager


class Controller:
    """Orchestrates the control pipeline: deadzone -> smoothing -> mixing.

    Converts high-level ControlCommands into low-level MotorCommands.
    """

    def __init__(self, estop: EStopManager | None = None):
        self._throttle_smoother = Smoother(THROTTLE_RAMP_RATE)
        self._steering_smoother = Smoother(STEERING_RAMP_RATE)
        self._estop = estop

    def process(self, cmd: ControlCommand, dt: float) -> MotorCommand:
        """Process a control command through the full pipeline.

        Args:
            cmd: High-level control command.
            dt: Time step in seconds since last call.

        Returns:
            Low-level motor command ready for hardware.
        """
        # 0. Emergency stop check
        if self._estop is not None:
            if cmd.emergency_stop:
                self._estop.activate()
            if self._estop.is_active():
                return MotorCommand(left=0, right=0, timestamp=cmd.timestamp)

        # 1. Apply deadzone
        throttle = apply_deadzone(cmd.throttle, DEADZONE_THRESHOLD)
        steering = apply_deadzone(cmd.steering, DEADZONE_THRESHOLD)

        # 2. Smooth (acceleration ramp)
        throttle = self._throttle_smoother.smooth(throttle, dt)
        steering = self._steering_smoother.smooth(steering, dt)

        # 3. Mix into motor values
        left, right = mix(throttle, steering)

        return MotorCommand(left=left, right=right, timestamp=cmd.timestamp)
