"""Deadzone filtering for input values."""

import math


def apply_deadzone(value: float, threshold: float) -> float:
    """Apply a deadzone to an input value.

    Values with absolute magnitude below the threshold snap to 0.
    Values above are rescaled so the output still spans the full range.

    Args:
        value: Input value (typically -1.0 to 1.0).
        threshold: Deadzone threshold (0.0 to 1.0).

    Returns:
        Filtered value with deadzone applied.
    """
    if abs(value) < threshold:
        return 0.0
    sign = math.copysign(1.0, value)
    return sign * (abs(value) - threshold) / (1.0 - threshold)
