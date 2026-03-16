"""Unit tests for deadzone filtering."""

from src.control.deadzone import apply_deadzone


class TestDeadzone:
    def test_below_threshold_returns_zero(self):
        """Values within deadzone snap to 0."""
        assert apply_deadzone(0.05, 0.1) == 0.0
        assert apply_deadzone(0.01, 0.1) == 0.0

    def test_at_threshold_returns_zero(self):
        """Exact threshold value returns 0 (strict less-than comparison)."""
        # The function uses abs(value) < threshold, so at exactly threshold
        # it should NOT be zero — it should be rescaled.
        # But threshold rescaled = (threshold - threshold) / (1 - threshold) = 0.0
        result = apply_deadzone(0.1, 0.1)
        assert result == 0.0

    def test_above_threshold_rescaled(self):
        """Values above threshold are rescaled to maintain full range."""
        # With threshold=0.1 and value=0.55:
        # output = (0.55 - 0.1) / (1.0 - 0.1) = 0.45 / 0.9 = 0.5
        result = apply_deadzone(0.55, 0.1)
        assert abs(result - 0.5) < 1e-9

    def test_negative_values(self):
        """Negative values work correctly (mirror of positive)."""
        result = apply_deadzone(-0.55, 0.1)
        assert abs(result - (-0.5)) < 1e-9

        # Negative value below threshold
        assert apply_deadzone(-0.05, 0.1) == 0.0

    def test_full_range(self):
        """Value of 1.0 maps to 1.0, -1.0 maps to -1.0."""
        assert abs(apply_deadzone(1.0, 0.1) - 1.0) < 1e-9
        assert abs(apply_deadzone(-1.0, 0.1) - (-1.0)) < 1e-9
