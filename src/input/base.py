"""Abstract base class for input controllers."""

from abc import ABC, abstractmethod

from src.models.commands import ControlCommand


class InputController(ABC):
    """Base class for all input controllers."""

    @abstractmethod
    def read(self) -> ControlCommand:
        """Return current command. Non-blocking."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return True if the controller is connected and operational."""
