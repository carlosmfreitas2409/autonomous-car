"""Unit tests for the acceleration ramp smoother."""

from src.control.smoother import Smoother


class TestSmoother:
    def test_step_input_ramps_gradually(self):
        """A step from 0 to 1.0 should not jump instantly."""
        s = Smoother(max_rate_per_second=2.0)
        # dt=0.1 -> max change = 0.2
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
        # Drive to target first
        for _ in range(10):
            s.smooth(1.0, 0.1)
        # Now it should stay at 1.0
        result = s.smooth(1.0, 0.1)
        assert abs(result - 1.0) < 1e-9

    def test_reset(self):
        s = Smoother(max_rate_per_second=2.0)
        s.smooth(1.0, 1.0)  # jumps to 1.0
        s.reset()
        # After reset, internal value is 0, so next smooth starts from 0
        result = s.smooth(1.0, 0.1)
        assert result == 0.2

    def test_ramp_down(self):
        s = Smoother(max_rate_per_second=2.0)
        s.smooth(1.0, 1.0)  # at 1.0
        result = s.smooth(0.0, 0.1)  # max change = 0.2 downward
        assert abs(result - 0.8) < 1e-9
