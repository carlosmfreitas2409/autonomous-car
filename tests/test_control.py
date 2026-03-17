"""Unit tests for the control pipeline: smoother, e-stop, watchdog, controller."""

import math
import time

from src.control import Controller, EStopManager, CommandWatchdog, Smoother


def _apply_deadzone(value: float, threshold: float) -> float:
    """Local copy for testing — avoids importing pygame via src.input."""
    if abs(value) < threshold:
        return 0.0
    sign = math.copysign(1.0, value)
    return sign * (abs(value) - threshold) / (1.0 - threshold)


# --- Smoother ---

class TestSmoother:
    def test_step_input_ramps_gradually(self):
        """A step from 0 to 1.0 should not jump instantly."""
        s = Smoother(max_rate_per_second=2.0)
        result = s.smooth(1.0, 0.1)
        assert result == 0.2

    def test_reaches_target_over_time(self):
        s = Smoother(max_rate_per_second=2.0)
        value = 0.0
        for _ in range(10):
            value = s.smooth(1.0, 0.1)
        assert abs(value - 1.0) < 1e-9

    def test_already_at_target_stays(self):
        s = Smoother(max_rate_per_second=2.0)
        for _ in range(10):
            s.smooth(1.0, 0.1)
        result = s.smooth(1.0, 0.1)
        assert abs(result - 1.0) < 1e-9

    def test_reset(self):
        s = Smoother(max_rate_per_second=2.0)
        s.smooth(1.0, 1.0)
        s.reset()
        result = s.smooth(1.0, 0.1)
        assert result == 0.2

    def test_ramp_down(self):
        s = Smoother(max_rate_per_second=2.0)
        s.smooth(1.0, 1.0)  # at 1.0
        result = s.smooth(0.0, 0.1)
        assert abs(result - 0.8) < 1e-9


# --- Deadzone ---

class TestDeadzone:
    def test_below_threshold_returns_zero(self):
        assert _apply_deadzone(0.05, 0.1) == 0.0
        assert _apply_deadzone(0.01, 0.1) == 0.0

    def test_at_threshold_returns_zero(self):
        result = _apply_deadzone(0.1, 0.1)
        assert result == 0.0

    def test_above_threshold_rescaled(self):
        result = _apply_deadzone(0.55, 0.1)
        assert abs(result - 0.5) < 1e-9

    def test_negative_values(self):
        result = _apply_deadzone(-0.55, 0.1)
        assert abs(result - (-0.5)) < 1e-9
        assert _apply_deadzone(-0.05, 0.1) == 0.0

    def test_full_range(self):
        assert abs(_apply_deadzone(1.0, 0.1) - 1.0) < 1e-9
        assert abs(_apply_deadzone(-1.0, 0.1) - (-1.0)) < 1e-9


# --- EStopManager ---

def test_estop_initial_state():
    estop = EStopManager()
    assert not estop.is_active()


def test_estop_activate():
    estop = EStopManager()
    estop.activate()
    assert estop.is_active()


def test_estop_reset():
    estop = EStopManager()
    estop.activate()
    assert estop.is_active()
    estop.reset()
    assert not estop.is_active()


# --- CommandWatchdog ---

def test_watchdog_not_expired_after_feed():
    wd = CommandWatchdog(timeout=1.0)
    wd.feed()
    assert not wd.is_expired()


def test_watchdog_expires():
    wd = CommandWatchdog(timeout=0.05)
    time.sleep(0.1)
    assert wd.is_expired()


# --- Controller ---

class TestController:
    def test_zero_input_zero_output(self):
        estop = EStopManager()
        controller = Controller(estop=estop)
        result = controller.process(0.0, 0.0, False, dt=0.1)
        assert result == (0.0, 0.0)

    def test_full_throttle_ramps(self):
        estop = EStopManager()
        controller = Controller(estop=estop)
        throttle, steering = controller.process(1.0, 0.0, False, dt=10.0)
        assert throttle > 0.0
        assert steering == 0.0

    def test_estop_zeroes_output(self):
        estop = EStopManager()
        controller = Controller(estop=estop)
        result = controller.process(1.0, 0.5, True, dt=10.0)
        assert result == (0.0, 0.0)

    def test_estop_resets_smoothers(self):
        """After e-stop release, throttle should ramp from zero."""
        estop = EStopManager()
        controller = Controller(estop=estop)

        # Build up speed
        controller.process(1.0, 0.0, False, dt=10.0)

        # E-stop (resets smoothers)
        controller.process(1.0, 0.0, True, dt=0.1)

        # Resume — should start from zero due to smoother reset
        estop.reset()
        throttle, _ = controller.process(1.0, 0.0, False, dt=0.01)
        assert throttle < 0.5  # ramping up, not full yet
