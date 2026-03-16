"""Simulated vehicle backed by the bicycle physics model."""

import time

from src.models.commands import MotorCommand
from src.models.state import VehicleState
from src.simulation.physics import BicyclePhysics
from src.vehicle.base import VehicleInterface


class SimulatedVehicle(VehicleInterface):
    """A simulated vehicle that uses BicyclePhysics for motion."""

    def __init__(self):
        self._physics = BicyclePhysics()
        self._last_time: float = time.monotonic()
        self._last_cmd = MotorCommand(left=0, right=0)

    def send_command(self, cmd: MotorCommand) -> None:
        """Send a motor command; advances the physics simulation.

        The motor command's left value is interpreted as throttle (0..255)
        and right value as steering (-255..255) for the bicycle model.
        """
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._last_cmd = cmd

        # Map motor command back to normalized values for physics
        throttle = max(0.0, cmd.left / 255.0)
        steering = cmd.right / 255.0
        self._physics.update(throttle, steering, dt)

    def get_state(self) -> VehicleState:
        """Return the current vehicle state from the physics engine."""
        state = self._physics.get_state()
        state.left_motor = self._last_cmd.left
        state.right_motor = self._last_cmd.right
        return state

    def stop(self) -> None:
        """Immediately stop the vehicle."""
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._last_cmd = MotorCommand(left=0, right=0)
        self._physics.update(0, 0, dt)

    def close(self) -> None:
        """No-op for simulated vehicle."""
        pass
