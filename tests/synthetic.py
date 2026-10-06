"""Crop fixtures, so both readers are testable with no camera and no model.

Drawn rather than photographed on purpose: a test that needs a JPEG of a real
traffic light can only be written after a shoot, and a test that needs the
model can only be run after a training run. These fixtures reproduce the
*properties* the readers depend on - a three-cell stack, a blown-out core, a
halo that spills past the housing, a polygon with a letter in it - and nothing
else.
"""

import cv2 as cv
import numpy as np

from perception.traffic_light import SPACING_OVER_RADIUS

HOUSING_MARGIN_X = 1.48
HOUSING_MARGIN_TOP = 1.53
HOUSING_MARGIN_BOTTOM = 1.15

BODY = (26, 26, 26)
COVER = (168, 166, 164)
LIT_COLOUR = {
    "red": (55, 55, 235),
    "yellow": (60, 215, 240),
    "green": (85, 215, 95),
}


def traffic_light_crop(state: str = "red", radius: int = 34, margin: float = 0.30,
                       background: int = 70, bloom: bool = True,
                       lamp_colour: str = None) -> np.ndarray:
    """A housing crop with three cells, one of them optionally lit.

    `state` is "red", "yellow", "green" or "off". A lit lamp gets a blown-out
    white core and a coloured halo that reaches past the housing, because that
    is what the camera does and what the reader has to survive.

    `lamp_colour` lights that cell in another lamp's colour, or "white" for
    none: the middle cell glowing red is a red lamp whose stack was fitted one
    cell too high.
    """
    spacing = SPACING_OVER_RADIUS * radius
    housing_w = int(2 * HOUSING_MARGIN_X * radius)
    housing_h = int((HOUSING_MARGIN_TOP + HOUSING_MARGIN_BOTTOM) * radius + 2 * spacing)
    pad_x, pad_y = int(housing_w * margin), int(housing_h * margin)

    crop = np.full((housing_h + 2 * pad_y, housing_w + 2 * pad_x, 3), background, np.uint8)
    cv.rectangle(crop, (pad_x, pad_y), (pad_x + housing_w, pad_y + housing_h), BODY, -1)

    cx = pad_x + housing_w // 2
    centres = [int(pad_y + HOUSING_MARGIN_TOP * radius + i * spacing) for i in range(3)]
    index = {"red": 0, "yellow": 1, "green": 2}.get(state)

    for position, cy in enumerate(centres):
        if position == index:
            continue
        cv.circle(crop, (cx, cy), radius, COVER, -1)

    if index is not None:
        colour = LIT_COLOUR.get(lamp_colour or {0: "red", 1: "yellow", 2: "green"}[index],
                                (235, 235, 235))
        cy = centres[index]
        if bloom:
            halo = np.zeros(crop.shape[:2], np.float32)
            cv.circle(halo, (cx, cy), int(radius * 1.15), 1.0, -1)
            blur = max(3, int(radius * 1.6) | 1)
            halo = cv.GaussianBlur(halo, (blur, blur), 0)
            halo = np.clip(halo * 2.2, 0, 1)[:, :, None]
            tint = np.float32(colour)[None, None, :]
            crop[:] = np.clip(crop.astype(np.float32) * (1 - halo) + tint * halo,
                              0, 255).astype(np.uint8)
        cv.circle(crop, (cx, cy), radius, colour, -1)
        cv.circle(crop, (cx, cy), int(radius * 0.72), (255, 255, 255), -1)
    return crop


def flooded_crop(state: str = "yellow", radius: int = 34, margin: float = 0.30) -> np.ndarray:
    """A lamp bright enough to erase its own housing.

    Past a certain brightness the glow swallows the covers either side and
    there is no three-cell stack left to find - which is what the middle lamp
    does to *both* of its neighbours, and why yellow is the state the main
    reader structurally cannot see. This is the crop that exercises the
    blown-core fallback.
    """
    crop = traffic_light_crop(state, radius=radius, margin=margin, background=45)
    index = {"red": 0, "yellow": 1, "green": 2}[state]
    height, width = crop.shape[:2]
    spacing = SPACING_OVER_RADIUS * radius
    pad_y = int((HOUSING_MARGIN_TOP + HOUSING_MARGIN_BOTTOM) * radius + 2 * spacing) * margin
    cy = int(pad_y + HOUSING_MARGIN_TOP * radius + index * spacing)

    # The flood has to reach far enough to lift the housing body to the same
    # brightness as the covers sitting in it. That is what removes the contrast
    # the cell finder needs, and merely adding a bright core does not do it.
    flood = np.zeros((height, width), np.float32)
    cv.circle(flood, (width // 2, cy), int(radius * 2.6), 1.0, -1)
    blur = max(3, int(radius * 2.6) | 1)
    flood = np.clip(cv.GaussianBlur(flood, (blur, blur), 0) * 4.5, 0, 1)[:, :, None]
    colour = np.float32(LIT_COLOUR[state])[None, None, :]
    lifted = np.maximum(crop.astype(np.float32), colour * 0.92 + 40.0)
    washed = crop.astype(np.float32) * (1 - flood) + lifted * flood

    core = np.zeros((height, width), np.float32)
    cv.circle(core, (width // 2, cy), int(radius * 1.3), 1.0, -1)
    core = np.clip(cv.GaussianBlur(core, (blur, blur), 0) * 2.2, 0, 1)[:, :, None]
    washed = washed * (1 - core) + 255.0 * core
    return np.clip(washed, 0, 255).astype(np.uint8)


def warm_room(crop: np.ndarray, strength: float = 0.35) -> np.ndarray:
    """Tint a crop the way the venue's stage lighting tints everything in it.

    This is the M2 case in one line: the unlit covers pick up genuine
    saturation, and a colour-fraction gate calls the housing RED.
    """
    warm = np.float32([0.75, 0.95, 1.30])
    tinted = crop.astype(np.float32) * (1 - strength + strength * warm)
    return np.clip(tinted, 0, 255).astype(np.uint8)


def delivery_sign_crop(variant: str = "A", height: int = 160, margin: float = 0.12,
                       background: int = 60, rotation: float = 0.0) -> np.ndarray:
    """A polygon sign face with its letter, cropped the way the reader wants it.

    Pentagon B, hexagon A, heptagon C - the frozen artwork. The crop is the
    sign's box grown by `margin`, matching delivery_variant.CROP_MARGIN.
    """
    sides = {"B": 5, "A": 6, "C": 7}[variant.upper()]
    pad = int(height * margin)
    size = height
    canvas = np.full((size + 2 * pad, size + 2 * pad, 3), background, np.uint8)

    centre = np.array([size / 2 + pad, size / 2 + pad])
    outer = size / 2
    start = np.deg2rad(rotation) - np.pi / 2
    points = np.array([[centre[0] + outer * np.cos(start + 2 * np.pi * i / sides),
                        centre[1] + outer * np.sin(start + 2 * np.pi * i / sides)]
                       for i in range(sides)], np.int32)
    cv.fillPoly(canvas, [points], (246, 246, 246))
    cv.polylines(canvas, [points], True, (18, 18, 18), max(1, int(height * 0.02)))

    scale = height / 130.0
    letter = variant.upper()
    (text_w, text_h), _ = cv.getTextSize(letter, cv.FONT_HERSHEY_DUPLEX, 2.4 * scale,
                                         max(1, int(5 * scale)))
    cv.putText(canvas, letter,
               (int(centre[0] - text_w / 2), int(centre[1] + text_h / 2)),
               cv.FONT_HERSHEY_DUPLEX, 2.4 * scale, (12, 12, 12), max(1, int(5 * scale)))
    return canvas


def glare_crop(position: float = 0.5, radius_fraction: float = 0.18,
               width: int = 120, height: int = 280, margin: float = 0.30) -> np.ndarray:
    """A saturated blob at a chosen height in an otherwise featureless box.

    No cells, no housing - just a bright thing where a lamp would be. What the
    blown-core fallback is allowed to make of it is the whole question.
    """
    crop = np.full((height, width, 3), 60, np.uint8)
    inset = margin / (1 + 2 * margin)
    top, box_height = inset * height, (1 - 2 * inset) * height
    centre = int(top + position * box_height)
    cv.circle(crop, (width // 2, centre), max(2, int(radius_fraction * height)),
              (255, 255, 255), -1)
    return crop


def blank_crop(width: int = 120, height: int = 280, value: int = 90) -> np.ndarray:
    return np.full((height, width, 3), value, np.uint8)


def lamp_pair_crop(width: int = 120, height: int = 280, radius: int = 34) -> np.ndarray:
    """Two bright lamps stacked - a stage rig, not a traffic light.

    Three is never drawn here, because three *is* the light's silhouette and a
    fixture that rejected it would reject the real thing.
    """
    crop = np.full((height, width, 3), 30, np.uint8)
    for cy in (height // 3, 2 * height // 3):
        cv.circle(crop, (width // 2, cy), radius, (250, 250, 250), -1)
    return crop
