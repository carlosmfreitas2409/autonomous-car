"""Differential drive motor mixing."""


def mix(throttle: float, steering: float) -> tuple[int, int]:
    """Mix throttle and steering into left/right motor values.

    Uses differential drive: left = throttle + steering, right = throttle - steering.
    Positive steering turns right (left wheel faster).

    If either raw value exceeds 1.0 in magnitude, both are scaled down
    proportionally to preserve the steering ratio.

    Final values are clamped to [-1.0, 1.0], scaled to [-255, 255], and
    cast to int.

    Args:
        throttle: Throttle input, -1.0 to 1.0.
        steering: Steering input, -1.0 to 1.0.

    Returns:
        Tuple of (left, right) motor PWM values as ints in [-255, 255].
    """
    left = throttle + steering
    right = throttle - steering

    # Scale both proportionally if either exceeds [-1, 1]
    max_mag = max(abs(left), abs(right))
    if max_mag > 1.0:
        left /= max_mag
        right /= max_mag

    # Clamp and scale
    left = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))

    return (int(left * 255), int(right * 255))
