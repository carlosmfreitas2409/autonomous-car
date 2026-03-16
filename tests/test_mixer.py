"""Unit tests for the motor mixer."""

from src.control.motor_mixer import mix


class TestMix:
    def test_full_throttle_no_steering(self):
        throttle_pwm, steering_pwm = mix(1.0, 0.0)
        assert throttle_pwm == 255
        assert steering_pwm == 0

    def test_no_throttle_full_steering(self):
        throttle_pwm, steering_pwm = mix(0.0, 1.0)
        assert throttle_pwm == 0
        assert steering_pwm == 255

    def test_zero_input(self):
        throttle_pwm, steering_pwm = mix(0.0, 0.0)
        assert throttle_pwm == 0
        assert steering_pwm == 0

    def test_negative_throttle_clamped(self):
        """Negative throttle is clamped to 0 (no reverse)."""
        throttle_pwm, steering_pwm = mix(-1.0, 0.0)
        assert throttle_pwm == 0

    def test_negative_steering(self):
        throttle_pwm, steering_pwm = mix(0.5, -0.5)
        assert throttle_pwm == 127
        assert steering_pwm == -127

    def test_values_clamped(self):
        throttle_pwm, steering_pwm = mix(2.0, 2.0)
        assert throttle_pwm == 255
        assert steering_pwm == 255
