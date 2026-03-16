"""Abstract base class for vehicle interfaces."""

from abc import ABC, abstractmethod

from src.models.commands import MotorCommand
from src.models.state import VehicleState


class VehicleInterface(ABC):
    """Abstract interface for controlling a vehicle (real or simulated)."""

    @abstractmethod
    def send_command(self, cmd: MotorCommand) -> None:
        """Send a motor command to the vehicle."""
        ...

    @abstractmethod
    def get_state(self) -> VehicleState:
        """Return the current vehicle state."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Stop all motors immediately."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release any resources held by this vehicle."""
        ...
