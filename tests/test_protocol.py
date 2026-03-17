"""Unit tests for the serial protocol — no real hardware required."""

from src.vehicle.protocol import SerialProtocol


class TestFormatMotorCommand:
    """Verify M <left> <right> format."""

    def test_positive_values(self):
        assert SerialProtocol.format_motor(128, 64) == "M 128 64"

    def test_negative_values(self):
        assert SerialProtocol.format_motor(-100, -200) == "M -100 -200"

    def test_zero_values(self):
        assert SerialProtocol.format_motor(0, 0) == "M 0 0"

    def test_mixed_values(self):
        assert SerialProtocol.format_motor(128, -64) == "M 128 -64"


class TestFormatEstop:
    """Verify E format."""

    def test_estop_format(self):
        assert SerialProtocol.format_estop() == "E"


class TestFormatHeartbeat:
    """Verify P format."""

    def test_heartbeat_format(self):
        assert SerialProtocol.format_heartbeat() == "P"


class TestParseSensorData:
    """Parse S ... lines correctly."""

    def test_valid_sensor_line(self):
        result = SerialProtocol.parse_sensor_data("S 10.5 20.3 15.0 90.0")
        assert result == {
            "dist_front": 10.5,
            "dist_left": 20.3,
            "dist_right": 15.0,
            "imu_heading": 90.0,
        }

    def test_integer_values(self):
        result = SerialProtocol.parse_sensor_data("S 10 20 15 90")
        assert result == {
            "dist_front": 10.0,
            "dist_left": 20.0,
            "dist_right": 15.0,
            "imu_heading": 90.0,
        }

    def test_zero_values(self):
        result = SerialProtocol.parse_sensor_data("S 0 0 0 0")
        assert result == {
            "dist_front": 0.0,
            "dist_left": 0.0,
            "dist_right": 0.0,
            "imu_heading": 0.0,
        }


class TestParseInvalidSensorData:
    """Handle malformed input gracefully."""

    def test_empty_string(self):
        assert SerialProtocol.parse_sensor_data("") == {}

    def test_wrong_prefix(self):
        assert SerialProtocol.parse_sensor_data("X 1 2 3 4") == {}

    def test_too_few_fields(self):
        assert SerialProtocol.parse_sensor_data("S 10 20") == {}

    def test_too_many_fields(self):
        assert SerialProtocol.parse_sensor_data("S 1 2 3 4 5") == {}

    def test_non_numeric_values(self):
        assert SerialProtocol.parse_sensor_data("S a b c d") == {}

    def test_error_line(self):
        assert SerialProtocol.parse_sensor_data("ERR 42") == {}

    def test_motor_command_line(self):
        assert SerialProtocol.parse_sensor_data("M 100 200") == {}
