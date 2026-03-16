"""Simulated vehicle backed by the differential-drive physics engine."""

import time

from src.models.commands import MotorCommand
from src.models.state import VehicleState
from src.simulation.physics import DifferentialDrivePhysics
from src.vehicle.base import VehicleInterface


class SimulatedVehicle(VehicleInterface):
    """A simulated vehicle that uses DifferentialDrivePhysics for motion."""

    def __init__(self):
        self._physics = DifferentialDrivePhysics()
        self._last_time: float = time.monotonic()
        self._last_cmd = MotorCommand(left=0, right=0)

    def send_command(self, cmd: MotorCommand) -> None:
        """Send a motor command; advances the physics simulation."""
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._last_cmd = cmd
        self._physics.update(cmd.left, cmd.right, dt)

    def get_state(self) -> VehicleState:
        """Return the current vehicle state from the physics engine."""
        state = self._physics.get_state()
        state.left_motor = self._last_cmd.left
        state.right_motor = self._last_cmd.right
        return state

    def stop(self) -> None:
        """Brake the vehicle by sending zero power."""
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._last_cmd = MotorCommand(left=0, right=0)
        self._physics.update(0, 0, dt)

    def close(self) -> None:
        """No-op for simulated vehicle."""
        pass
