"""Serial communication protocol for Arduino vehicle control."""

import logging
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)


class SerialProtocol:
    """Handles serial communication with the Arduino motor controller.

    Message format (ASCII, newline-terminated):
        Python -> Arduino:
            Motor:     M <left> <right>\\n
            E-stop:    E\\n
            Heartbeat: P\\n
        Arduino -> Python:
            Sensor:    S <dist_front> <dist_left> <dist_right> <imu_heading>\\n
            Error:     ERR <code>\\n
    """

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        if serial is None:
            raise ImportError(
                "pyserial is required for serial communication. "
                "Install it with: pip install pyserial"
            )
        try:
            self._serial = serial.Serial(port, baudrate, timeout=0.01)
            logger.info("Opened serial port %s at %d baud", port, baudrate)
        except serial.SerialException as e:
            logger.error("Failed to open serial port %s: %s", port, e)
            raise

    # ---- Format helpers (class/static for testability) ----

    @staticmethod
    def format_motor(left: int, right: int) -> str:
        """Return the motor command string (without trailing newline)."""
        return f"M {left} {right}"

    @staticmethod
    def format_estop() -> str:
        """Return the e-stop command string."""
        return "E"

    @staticmethod
    def format_heartbeat() -> str:
        """Return the heartbeat command string."""
        return "P"

    # ---- Send methods ----

    def send_motor(self, left: int, right: int) -> None:
        """Send a motor command to the Arduino."""
        self._write(self.format_motor(left, right))

    def send_estop(self) -> None:
        """Send an emergency stop command."""
        self._write(self.format_estop())

    def send_heartbeat(self) -> None:
        """Send a heartbeat / ping."""
        self._write(self.format_heartbeat())

    # ---- Read methods ----

    def read_response(self) -> Optional[str]:
        """Non-blocking read of one line from the Arduino.

        Returns the stripped line, or None if nothing available.
        """
        try:
            if self._serial.in_waiting:
                raw = self._serial.readline()
                if raw:
                    return raw.decode("ascii", errors="replace").strip()
        except serial.SerialException as e:
            logger.error("Serial read error: %s", e)
        return None

    @staticmethod
    def parse_sensor_data(line: str) -> dict[str, float]:
        """Parse an ``S ...`` sensor line into a dict.

        Expected format: ``S <dist_front> <dist_left> <dist_right> <imu_heading>``

        Returns an empty dict if the line is malformed.
        """
        try:
            parts = line.strip().split()
            if len(parts) != 5 or parts[0] != "S":
                return {}
            return {
                "dist_front": float(parts[1]),
                "dist_left": float(parts[2]),
                "dist_right": float(parts[3]),
                "imu_heading": float(parts[4]),
            }
        except (ValueError, IndexError):
            return {}

    # ---- Lifecycle ----

    def close(self) -> None:
        """Close the serial port."""
        try:
            if self._serial.is_open:
                self._serial.close()
                logger.info("Serial port closed")
        except serial.SerialException as e:
            logger.error("Error closing serial port: %s", e)

    # ---- Internal ----

    def _write(self, message: str) -> None:
        """Write a newline-terminated ASCII message to the serial port."""
        try:
            self._serial.write(f"{message}\n".encode("ascii"))
        except serial.SerialException as e:
            logger.error("Serial write error: %s", e)
