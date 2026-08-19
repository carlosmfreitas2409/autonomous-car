"""Synthetic traffic lights, so colour logic can be tested without a camera."""

import cv2 as cv
import numpy as np

from perception.traffic_light import LightState


LAMP_BGR = {
    LightState.RED: (0, 0, 255),
    LightState.YELLOW: (0, 200, 255),
    LightState.GREEN: (0, 255, 0),
}

LAYOUT = {
    2: (LightState.RED, LightState.GREEN),
    3: (LightState.RED, LightState.YELLOW, LightState.GREEN),
}

# How much of its colour an unlit lamp still shows.
_UNLIT_SCALE = 0.12


def render_light(
    state,
    *,
    n_lamps=3,
    vertical=True,
    lamp_px=40,
    housing=(30, 30, 30),
    clipped_core=False,
    brightness=1.0,
    blur=0,
    pad=0,
    lit_index=None,
):
    """Draw a traffic light housing with exactly one lamp lit."""
    if lit_index is None:
        lit_index = LAYOUT[n_lamps].index(state)

    long_px, short_px = lamp_px * n_lamps, lamp_px
    shape = (long_px, short_px) if vertical else (short_px, long_px)
    img = np.full((*shape, 3), housing, dtype=np.uint8)

    radius = max(1, int(lamp_px * 0.36))
    for i in range(n_lamps):
        along = int(lamp_px * (i + 0.5))
        centre = (short_px // 2, along) if vertical else (along, short_px // 2)

        if i == lit_index:
            colour = LAMP_BGR[state]
        else:
            base = LAMP_BGR[LAYOUT[n_lamps][i]]
            colour = tuple(int(c * _UNLIT_SCALE) for c in base)
        cv.circle(img, centre, radius, colour, -1)

        # A bright LED clips its centre to white; the colour survives only in
        # the halo. This is the case hue-from-the-middle gets wrong.
        if i == lit_index and clipped_core:
            cv.circle(img, centre, max(1, int(radius * 0.5)), (255, 255, 255), -1)

    if brightness != 1.0:
        img = np.clip(img.astype(np.float32) * brightness, 0, 255).astype(np.uint8)
    if blur:
        img = cv.GaussianBlur(img, (blur | 1, blur | 1), 0)
    if pad:
        img = cv.copyMakeBorder(img, pad, pad, pad, pad, cv.BORDER_CONSTANT, value=(70, 90, 60))
    return img
