"""Motor command mixing for bicycle model vehicle."""


def mix(throttle: float, steering: float) -> tuple[int, int]:
    """Convert throttle and steering to motor command values.

    For the bicycle model, left=throttle (0..255) and right=steering (-255..255).

    Args:
        throttle: Throttle input, 0.0 to 1.0 (forward only).
        steering: Steering input, -1.0 to 1.0.

    Returns:
        Tuple of (throttle_pwm, steering_pwm) as ints.
    """
    throttle = max(0.0, min(1.0, throttle))
    steering = max(-1.0, min(1.0, steering))

    return (int(throttle * 255), int(steering * 255))
