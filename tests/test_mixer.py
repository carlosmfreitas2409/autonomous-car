"""Unit tests for the motor mixer."""

from src.control.motor_mixer import mix


class TestMix:
    def test_full_throttle_no_steering(self):
        left, right = mix(1.0, 0.0)
        assert left == 255
        assert right == 255

    def test_no_throttle_full_right_steering(self):
        left, right = mix(0.0, 1.0)
        assert left == 255
        assert right == -255

    def test_zero_input(self):
        left, right = mix(0.0, 0.0)
        assert left == 0
        assert right == 0

    def test_overflow_scaling(self):
        """When throttle + steering > 1.0, both should scale down."""
        left, right = mix(1.0, 1.0)
        # Raw: left=2.0, right=0.0 -> scaled: left=1.0, right=0.0
        assert left == 255
        assert right == 0

    def test_negative_throttle(self):
        left, right = mix(-1.0, 0.0)
        assert left == -255
        assert right == -255

    def test_overflow_preserves_ratio(self):
        """Scaling should preserve the ratio between left and right."""
        left, right = mix(0.8, 0.5)
        # Raw: left=1.3, right=0.3 -> scale by 1/1.3
        # left=1.0, right=0.3/1.3 ~ 0.2307...
        assert left == 255
        assert right == int(0.3 / 1.3 * 255)
