"""Arduino-based vehicle implementation using serial communication."""

from src.simulation.physics import VehicleState
from src.vehicle.protocol import SerialProtocol


class ArduinoVehicle:
    """Vehicle backed by a real Arduino over serial."""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._protocol = SerialProtocol(port, baudrate)
        self._last_sensors: dict[str, float] = {}

    def send_command(self, throttle: float, steering: float) -> None:
        """Convert throttle/steering to PWM and send to Arduino."""
        left_pwm = int(max(0.0, min(1.0, throttle)) * 255)
        right_pwm = int(max(-1.0, min(1.0, steering)) * 255)
        self._protocol.send_motor(left_pwm, right_pwm)

    def get_state(self) -> VehicleState:
        while True:
            line = self._protocol.read_response()
            if line is None:
                break
            sensors = self._protocol.parse_sensor_data(line)
            if sensors:
                self._last_sensors = sensors

        return VehicleState(sensors=dict(self._last_sensors))

    def stop(self) -> None:
        self._protocol.send_estop()

    def close(self) -> None:
        self.stop()
        self._protocol.close()
