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
            Motor:     M <throttle_pwm> <steering_pwm>\\n
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

    @staticmethod
    def format_motor(throttle: int, steering: int) -> str:
        return f"M {throttle} {steering}"

    @staticmethod
    def format_estop() -> str:
        return "E"

    @staticmethod
    def format_heartbeat() -> str:
        return "P"

    def send_motor(self, throttle: int, steering: int) -> None:
        self._write(self.format_motor(throttle, steering))

    def send_estop(self) -> None:
        self._write(self.format_estop())

    def send_heartbeat(self) -> None:
        self._write(self.format_heartbeat())

    def read_response(self) -> Optional[str]:
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

    def close(self) -> None:
        try:
            if self._serial.is_open:
                self._serial.close()
                logger.info("Serial port closed")
        except serial.SerialException as e:
            logger.error("Error closing serial port: %s", e)

    def _write(self, message: str) -> None:
        try:
            self._serial.write(f"{message}\n".encode("ascii"))
        except serial.SerialException as e:
            logger.error("Serial write error: %s", e)
