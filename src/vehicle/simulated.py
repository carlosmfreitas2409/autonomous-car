"""Simulated vehicle backed by the bicycle physics model."""

import time

from src.simulation.physics import BicyclePhysics, VehicleState


class SimulatedVehicle:
    """Uses BicyclePhysics to simulate the car."""

    def __init__(self):
        self._physics = BicyclePhysics()
        self._last_time: float = time.monotonic()

    def send_command(self, throttle: float, steering: float) -> None:
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._physics.update(throttle, steering, dt)

    def get_state(self) -> VehicleState:
        return self._physics.get_state()

    def stop(self) -> None:
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._physics.update(0, 0, dt)

    def close(self) -> None:
        pass
