"""Unit tests for the Controller class."""

from src.control.controller import Controller
from src.models.commands import ControlCommand, MotorCommand
from src.safety.emergency_stop import EStopManager


class TestController:
    def test_zero_input_zero_output(self):
        """ControlCommand(0, 0) produces MotorCommand(0, 0)."""
        controller = Controller()
        cmd = ControlCommand(throttle=0.0, steering=0.0)
        result = controller.process(cmd, dt=0.1)
        assert result.left == 0
        assert result.right == 0

    def test_full_throttle_produces_motor_output(self):
        """ControlCommand(1, 0) produces both motors positive."""
        controller = Controller()
        # Use a large dt so the smoother doesn't limit the ramp too much
        cmd = ControlCommand(throttle=1.0, steering=0.0)
        result = controller.process(cmd, dt=10.0)
        assert result.left > 0
        assert result.right > 0

    def test_estop_zeroes_output(self):
        """When estop is active, output is always (0, 0)."""
        estop = EStopManager()
        controller = Controller(estop=estop)

        # Activate estop
        estop.activate()

        cmd = ControlCommand(throttle=1.0, steering=0.5)
        result = controller.process(cmd, dt=10.0)
        assert result.left == 0
        assert result.right == 0
