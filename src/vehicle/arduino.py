"""Arduino-based vehicle implementation using serial communication."""

import logging

from src.models.commands import MotorCommand
from src.models.state import VehicleState
from src.protocol.serial_protocol import SerialProtocol
from src.vehicle.base import VehicleInterface

logger = logging.getLogger(__name__)


class ArduinoVehicle(VehicleInterface):
    """Vehicle interface backed by a real Arduino over serial."""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._protocol = SerialProtocol(port, baudrate)
        self._last_sensors: dict[str, float] = {}

    def send_command(self, cmd: MotorCommand) -> None:
        """Send a motor command to the Arduino."""
        self._protocol.send_motor(cmd.left, cmd.right)

    def get_state(self) -> VehicleState:
        """Read serial for sensor data and return a VehicleState.

        Position is not tracked — only sensor data is populated.
        """
        # Drain available lines, keeping the latest sensor reading.
        while True:
            line = self._protocol.read_response()
            if line is None:
                break
            sensors = self._protocol.parse_sensor_data(line)
            if sensors:
                self._last_sensors = sensors

        return VehicleState(sensors=dict(self._last_sensors))

    def stop(self) -> None:
        """Send an emergency stop command."""
        self._protocol.send_estop()

    def close(self) -> None:
        """Stop the vehicle and close the serial connection."""
        self.stop()
        self._protocol.close()
