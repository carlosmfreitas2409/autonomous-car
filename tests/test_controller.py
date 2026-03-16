"""Unit tests for the Controller class."""

from src.control.controller import Controller
from src.models.commands import ControlCommand
from src.safety.emergency_stop import EStopManager


class TestController:
    def test_zero_input_zero_output(self):
        controller = Controller()
        cmd = ControlCommand(throttle=0.0, steering=0.0)
        result = controller.process(cmd, dt=0.1)
        assert result.left == 0
        assert result.right == 0

    def test_full_throttle_produces_output(self):
        controller = Controller()
        cmd = ControlCommand(throttle=1.0, steering=0.0)
        result = controller.process(cmd, dt=10.0)
        assert result.left > 0  # throttle PWM
        assert result.right == 0  # no steering

    def test_estop_zeroes_output(self):
        estop = EStopManager()
        controller = Controller(estop=estop)
        estop.activate()

        cmd = ControlCommand(throttle=1.0, steering=0.5)
        result = controller.process(cmd, dt=10.0)
        assert result.left == 0
        assert result.right == 0

    def test_estop_resets_smoothers(self):
        """After e-stop release, throttle should ramp from zero, not resume."""
        estop = EStopManager()
        controller = Controller(estop=estop)

        # Build up speed
        cmd = ControlCommand(throttle=1.0, steering=0.0)
        controller.process(cmd, dt=10.0)

        # E-stop (resets smoothers)
        estop.activate()
        controller.process(ControlCommand(emergency_stop=True), dt=0.1)

        # Resume — should start from zero due to smoother reset
        estop.reset()
        result = controller.process(ControlCommand(throttle=1.0), dt=0.01)
        assert result.left < 128  # not full throttle yet, ramping up
