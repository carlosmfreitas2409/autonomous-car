"""Read A, B or C out of a delivery-sign crop.

The detector reports one `delivery_sign` class. It does not try to tell the
three apart, because one extra polygon side at 30 px is not something a
detector head should be asked to bet a routing decision on: the previous
six-class model confused pentagon B for hexagon A 6 % of the time and heptagon
C for pentagon B 10 % of the time *on its own synthetic validation split*, the
easiest test it will ever face. A wrong B at range is a confident wrong
routing decision; a `delivery_sign` with the variant deferred is honest and
costs nothing, because the car does not need to know which bay until it is
close enough to turn in.

So the variant is read here, from the crop, by two cues that are deliberately
independent and deliberately threshold-free:

  * **How many sides.** Not `approxPolyDP`, whose epsilon is a knob that needs
    retuning at every apparent size. The distance from the face's centroid to
    its outline, sampled at 256 angles and divided by its own mean, puts all of
    a regular n-gon's energy in harmonic n. Rotation shifts phase only; scale
    cancels; harmonics 0 to 2 are dropped because they carry the mean, the
    centroid's offset inside a loose box, and perspective elongation.
  * **Which letter.** Hole count first, template second. The glyphs are
    *topologically* distinct - A has one counter, B has two, C has none - and
    counting holes needs no threshold and fails to "ambiguous" rather than to a
    confident wrong letter. A 32x32 correlation against glyphs rendered from
    the same artwork confirms it.

Segmenting the white face is the actual hard part, and the pilots that led to
this design disagreed with each other entirely because of it. Measured: without
upsampling first, segmentation fails on 100 % of crops under 64 px; with a
tolerance taken from the crop's global standard deviation it fails on 100 % of
crops at any size, because that deviation is dominated by the black outline and
the background and yields a tolerance that floods everything. A per-seed local
tolerance plus a sanity window on the resulting area fixed it.

If the two cues disagree the answer is None. The car then approaches a sign it
knows is a delivery sign and reads it again from closer.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2 as cv
import numpy as np

# Sides to variant. Frozen artwork: pentagon B, hexagon A, heptagon C.
SIDES_TO_VARIANT = {5: "B", 6: "A", 7: "C"}
VARIANT_TO_SIDES = {v: k for k, v in SIDES_TO_VARIANT.items()}

# Holes in the glyph. A has one counter, B has two, C has none.
HOLES_TO_VARIANT = {0: "C", 1: "A", 2: "B"}

# Signs shorter than this are not read at all - the reader reports
# `delivery_sign` and nothing more, and the car reads it again from closer.
# Set by tools/eval_delivery_variant.py, which measures where the fused answer
# first reaches 0.95 and stays there: 110 px, which for a 15 cm sign on this
# camera is about 1.4 m. Below it the reader mostly refuses rather than errs -
# wrong answers are under 2 % from 34 px up - but it is not accurate enough to
# turn into a bay on.
MIN_CROP_HEIGHT = 110

# Where the two cues swap places. Measured by tools/eval_delivery_variant.py:
# below this the silhouette is right more often than the letter, above it the
# letter is. It is only consulted when the two disagree. This settles the
# question the brief and the older write-up disagreed about: the silhouette
# survives further out, and by 110 px the letter has overtaken it.
CUE_CROSSOVER = 80

# Work at 96 px tall at least: below that the face has no outline to sample and
# the glyph has no holes to count. Capped, because past ~220 px the extra
# pixels only cost time.
_MIN_WORK_HEIGHT = 96
_MAX_WORK_HEIGHT = 224

# How much room to leave around the detector's box when cropping for this
# reader. Without it a box a few per cent tight cuts the polygon's vertices off
# and the outline the side count measures is partly the crop's own border -
# measured, that is what drops the side count from perfect on clean artwork to
# 0.80 at 224 px on rendered ones.
CROP_MARGIN = 0.12

# Where the face is seeded from, in coordinates of the sign's own box rather
# than the crop's. A polygon inscribed in its box touches that box near the
# middle of each edge and leaves the corners to the background, so the middle
# of an edge is white face and a corner is not. Seeding the centre instead
# would land on the letter as often as not.
_SEEDS = ((0.50, 0.08), (0.50, 0.92), (0.08, 0.50), (0.92, 0.50),
          (0.30, 0.14), (0.70, 0.86))

# A flood that covers less than a third or more than nineteen twentieths of the
# crop did not find the face: it found the letter, or it leaked into the
# background. The face itself measures 0.60 to 0.70 of its box on the artwork.
_FACE_AREA = (0.30, 0.95)

# Harmonics the side count is allowed to choose between, and the ones that are
# never looked at: 0 is the mean, 1 the centroid offset, 2 the elongation a
# perspective view adds.
# A pentagon, a hexagon and a heptagon are convex, so their outline hugs its
# own convex hull. Anything appreciably less convex than this is a room.
_MIN_SOLIDITY = 0.90

_HARMONICS = (5, 6, 7)
_ANGLES = 256

_GLYPH = 32
_ARTWORK = Path(__file__).resolve().parent.parent.parent / "assets" / "sign_cutouts" / "delivery_sign"


@dataclass
class VariantReading:
    variant: Optional[str] = None
    confidence: float = 0.0
    # Each cue's own answer, kept separately so a disagreement is visible
    # rather than averaged away.
    sides: Optional[int] = None
    sides_variant: Optional[str] = None
    sides_margin: float = 0.0
    holes: Optional[int] = None
    letter_variant: Optional[str] = None
    letter_score: float = 0.0
    letter_gap: float = 0.0
    reason: str = ""
    debug: dict = field(default_factory=dict)


def read_delivery_variant(crop: np.ndarray,
                          min_height: int = MIN_CROP_HEIGHT,
                          margin: float = CROP_MARGIN) -> VariantReading:
    """Read A / B / C from a delivery-sign crop, or refuse.

    `crop` is the detector's box grown by `margin` on every side, so the whole
    polygon is inside it with background around it. `min_height` is measured on
    the sign, not on the crop.
    """
    if crop is None or crop.size == 0 or min(crop.shape[:2]) < 6:
        return VariantReading(reason="crop too small")
    sign_height = crop.shape[0] / (1 + 2 * margin)
    if sign_height < min_height:
        return VariantReading(reason=f"sign {sign_height:.0f} px < {min_height} px floor")

    work = _to_work_height(crop)
    face = segment_face(work, margin)
    if face is None:
        return VariantReading(reason="no sign face segmented")

    reading = VariantReading()
    reading.debug["work"] = work
    reading.debug["face_mask"] = face

    sides, sides_margin, contour = count_sides(face)
    reading.sides, reading.sides_margin = sides, sides_margin
    reading.sides_variant = SIDES_TO_VARIANT.get(sides)
    reading.debug["contour"] = contour

    letter, glyph = read_letter(work, face)
    reading.holes = letter.get("holes")
    reading.letter_variant = letter.get("variant")
    reading.letter_score = letter.get("score", 0.0)
    reading.letter_gap = letter.get("gap", 0.0)
    reading.debug["glyph"] = glyph

    _fuse(reading, sign_height)
    return reading


# --------------------------------------------------------------------------
# segmentation - the part the pilots got wrong
# --------------------------------------------------------------------------

def segment_face(bgr: np.ndarray, margin: float = CROP_MARGIN) -> Optional[np.ndarray]:
    """The white sign face, flood-filled from six seeds near the sign's edges.

    Each seed brings its own tolerance, taken from the 5x5 patch around it. A
    tolerance from the whole crop's spread is dominated by the black outline
    and by whatever is behind the sign, and floods the entire image; the
    variation in a 5x5 patch of white card is the sensor noise on white card,
    which is exactly what the flood has to tolerate.

    A seed that lands outside the sign floods the background instead, and on a
    pale wall that flood is the right size and the wrong region. Three
    questions sort them out, and none of them is a threshold on brightness: the
    face contains the middle of the crop, it does not reach the crop's corners,
    and it is convex. A pentagon, a hexagon and a heptagon are all convex; a
    room is not.
    """
    height, width = bgr.shape[:2]
    blurred = cv.GaussianBlur(bgr, (3, 3), 0)
    # The sign's own box inside the margin-grown crop.
    low, span = margin / (1 + 2 * margin), 1 / (1 + 2 * margin)
    sign_area = float(height * width) * span * span
    inset = max(1, int(0.03 * min(height, width)))
    corners = ((inset, inset), (width - inset - 1, inset),
               (inset, height - inset - 1), (width - inset - 1, height - inset - 1))
    centre = (width // 2, height // 2)

    best = None
    for fx, fy in _SEEDS:
        seed = (int(np.clip((low + fx * span) * width, 1, width - 2)),
                int(np.clip((low + fy * span) * height, 1, height - 2)))
        tolerance = _local_tolerance(blurred, seed)
        mask = np.zeros((height + 2, width + 2), np.uint8)
        cv.floodFill(blurred.copy(), mask, seed, 255,
                     (tolerance,) * 3, (tolerance,) * 3,
                     cv.FLOODFILL_MASK_ONLY | cv.FLOODFILL_FIXED_RANGE | (255 << 8))

        # The flood stops at the letter, so fill the letter back in: the
        # outline this leaves is what the side count measures, and the region
        # is where the glyph search has to look.
        contours, _ = cv.findContours(mask[1:-1, 1:-1], cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        if not contours:
            continue
        outline = max(contours, key=cv.contourArea)
        solid = np.zeros((height, width), np.uint8)
        cv.drawContours(solid, [outline], -1, 255, cv.FILLED)

        covered = float(solid.sum()) / 255.0
        if not _FACE_AREA[0] * sign_area <= covered <= _FACE_AREA[1] * sign_area:
            continue
        if not solid[centre[1], centre[0]]:
            continue
        if any(solid[cy, cx] for cx, cy in corners):
            continue
        hull_area = cv.contourArea(cv.convexHull(outline))
        solidity = cv.contourArea(outline) / hull_area if hull_area > 0 else 0.0
        if solidity < _MIN_SOLIDITY:
            continue
        if best is None or covered > best[0]:
            best = (covered, solid)
    return None if best is None else best[1]


def _local_tolerance(bgr: np.ndarray, seed) -> int:
    x, y = seed
    patch = bgr[max(0, y - 2):y + 3, max(0, x - 2):x + 3].astype(np.float32)
    spread = float(patch.reshape(-1, patch.shape[-1]).std(axis=0).mean())
    return int(np.clip(3.0 * spread + 8, 8, 70))


# --------------------------------------------------------------------------
# cue 1: how many sides
# --------------------------------------------------------------------------

def count_sides(face: np.ndarray) -> tuple[Optional[int], float, Optional[np.ndarray]]:
    """Side count by radial harmonic energy, and how clearly it won.

    A regular n-gon's centroid-to-outline distance is a constant plus a cosine
    at exactly n cycles per turn. Dividing by the mean makes it scale-free and
    the harmonic's phase absorbs rotation, so a hexagon at 16 px and the same
    hexagon at 224 px, turned, score the same harmonic.

    The face is whitened first - scaled along its own principal axes, from the
    image moments of the mask, until its second moments are equal. A regular
    polygon is already isotropic, so this does nothing to a sign seen square
    on; a sign seen from an angle is stretched and sheared, and whitening is
    the undo for that. Measured, it lifts the side count from 0.83 to 0.93 at
    40 px and from 0.84 to 0.94 at 28 px, and changes nothing at 224.
    """
    contours, _ = cv.findContours(face, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if not contours:
        return None, 0.0, None
    contour = max(contours, key=cv.contourArea)
    if len(contour) < 16:
        return None, 0.0, contour

    # Every contour point, not the convex hull's vertices: the hull keeps only
    # the corners, and a radius function sampled at seven corners is a
    # seven-sided shape whatever the sign was.
    points = _whiten(contour.reshape(-1, 2).astype(np.float32), face)
    if points is None:
        return None, 0.0, contour

    radii = np.hypot(points[:, 0], points[:, 1])
    if radii.mean() <= 1e-3:
        return None, 0.0, contour
    angles = np.mod(np.arctan2(points[:, 1], points[:, 0]), 2 * np.pi)

    order = np.argsort(angles)
    grid = np.linspace(0, 2 * np.pi, _ANGLES, endpoint=False)
    # Wrap one sample around each end so the interpolation closes the circle.
    sampled = np.interp(grid,
                        np.concatenate(([angles[order][-1] - 2 * np.pi], angles[order],
                                        [angles[order][0] + 2 * np.pi])),
                        np.concatenate(([radii[order][-1]], radii[order], [radii[order][0]])))
    sampled = sampled / sampled.mean()

    spectrum = np.abs(np.fft.rfft(sampled))
    energies = {n: float(spectrum[n]) for n in _HARMONICS}
    ranked = sorted(energies.items(), key=lambda kv: -kv[1])
    best, runner_up = ranked[0], ranked[1]
    margin = best[1] / runner_up[1] if runner_up[1] > 1e-6 else float("inf")
    return best[0], float(min(margin, 99.0)), contour


def _whiten(points: np.ndarray, face: np.ndarray) -> Optional[np.ndarray]:
    """Centre a point set on the face and make the face's moments isotropic.

    The moments come from the filled mask rather than from the outline's
    points, because outline points are dense on long edges and sparse on short
    ones and would weight the estimate by perimeter instead of by area.
    """
    moments = cv.moments(face, binaryImage=True)
    if moments["m00"] <= 0:
        return None
    covariance = np.array([[moments["mu20"], moments["mu11"]],
                           [moments["mu11"], moments["mu02"]]]) / moments["m00"]
    values, vectors = np.linalg.eigh(covariance)
    if values.min() <= 1e-6 or not np.all(np.isfinite(values)):
        return None
    centre = np.array([moments["m10"] / moments["m00"],
                       moments["m01"] / moments["m00"]], np.float32)
    return (points - centre) @ (vectors / np.sqrt(values)) @ vectors.T


# --------------------------------------------------------------------------
# cue 2: which letter
# --------------------------------------------------------------------------

def read_letter(bgr: np.ndarray, face: np.ndarray) -> tuple[dict, Optional[np.ndarray]]:
    """The glyph's hole count, confirmed by correlation against the artwork."""
    glyph_mask = _glyph_mask(bgr, face)
    if glyph_mask is None:
        return {"holes": None, "variant": None}, None

    holes = _count_holes(glyph_mask)
    normalised = _normalise_glyph(glyph_mask)
    result = {"holes": holes, "variant": HOLES_TO_VARIANT.get(holes)}

    scores = {}
    for name, template in _templates().items():
        scores[name] = float((normalised * template).sum() /
                             (np.linalg.norm(normalised) * np.linalg.norm(template) + 1e-6))
    if scores:
        ranked = sorted(scores.items(), key=lambda kv: -kv[1])
        result["score"] = ranked[0][1]
        result["gap"] = ranked[0][1] - ranked[1][1]
        # The hole count decides; correlation is the check on it. A glyph whose
        # topology is unreadable at this size can still be matched by shape.
        if result["variant"] is None:
            result["variant"] = ranked[0][0]
        elif ranked[0][0] != result["variant"] and result["gap"] > 0.08:
            result["variant"] = None
    return result, normalised


def _glyph_mask(bgr: np.ndarray, face: np.ndarray) -> Optional[np.ndarray]:
    """The dark blob inside the face - the letter, and nothing else.

    "Dark" is relative to this face's own brightness, so a sign in shadow and
    the same sign under a stage lamp are read the same way.
    """
    gray = cv.cvtColor(bgr, cv.COLOR_BGR2GRAY)
    inside = cv.erode(face, cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3)))
    values = gray[inside > 0]
    if values.size < 40:
        return None
    white = float(np.percentile(values, 80))
    dark = float(np.percentile(values, 5))
    if white - dark < 18:
        return None
    level = (white + dark) / 2.0

    mask = ((gray < level) & (inside > 0)).astype(np.uint8)
    count, labels, stats, _ = cv.connectedComponentsWithStats(mask, 8)
    if count < 2:
        return None
    index = 1 + int(np.argmax(stats[1:, cv.CC_STAT_AREA]))
    x, y, w, h, area = stats[index]
    if area < 0.01 * face.size or w < 3 or h < 3:
        return None
    return (labels[y:y + h, x:x + w] == index).astype(np.uint8)


def _count_holes(glyph: np.ndarray) -> int:
    """Euler number of the glyph: components minus holes, and there is one
    component, so the hole count falls straight out. No threshold anywhere."""
    padded = cv.copyMakeBorder(glyph, 1, 1, 1, 1, cv.BORDER_CONSTANT, value=0)
    count, labels, stats, _ = cv.connectedComponentsWithStats(1 - padded, 4)
    # In the inverse, label 0 is the glyph itself and the component touching the
    # border is the world outside it. Everything else is a counter. Specks
    # smaller than a half percent of the glyph are JPEG, not a hole.
    outside = int(labels[0, 0])
    floor = max(2.0, 0.005 * float(glyph.sum()))
    return int(sum(1 for i in range(1, count)
                   if i != outside and stats[i, cv.CC_STAT_AREA] >= floor))


def _normalise_glyph(glyph: np.ndarray) -> np.ndarray:
    """A 32x32 glyph, letterboxed rather than stretched.

    Aspect ratio is a cue in its own right - measured on the artwork, A is
    1.03 high over wide, B 1.22 and C 1.32 - so stretching each glyph to fill
    the square would throw away the easiest difference between them.
    """
    height, width = glyph.shape
    scale = (_GLYPH - 4) / max(height, width)
    resized = cv.resize(glyph.astype(np.float32),
                        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
                        interpolation=cv.INTER_AREA)
    canvas = np.zeros((_GLYPH, _GLYPH), np.float32)
    y = (_GLYPH - resized.shape[0]) // 2
    x = (_GLYPH - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    canvas -= canvas.mean()
    return canvas


_TEMPLATE_CACHE: Optional[dict] = None


def _templates() -> dict:
    """Glyph templates rendered from the same artwork the signs are printed
    from. Missing artwork is not fatal: the hole count still works."""
    global _TEMPLATE_CACHE
    if _TEMPLATE_CACHE is not None:
        return _TEMPLATE_CACHE

    templates = {}
    for variant in ("a", "b", "c"):
        path = _ARTWORK / f"variant_{variant}.png"
        image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
        if image is None:
            continue
        # Composite onto mid grey: the artwork's transparent corners are
        # undefined colour, and a flood seeded near an edge would walk into
        # them if they happened to be white.
        bgr = image[:, :, :3].copy()
        bgr[image[:, :, 3] < 128] = (128, 128, 128)
        bgr = cv.resize(bgr, (192, 192))
        face = segment_face(bgr, margin=0.0)
        if face is None:
            continue
        mask = _glyph_mask(bgr, face)
        if mask is None:
            continue
        templates[variant.upper()] = _normalise_glyph(mask)
    _TEMPLATE_CACHE = templates
    return templates


# --------------------------------------------------------------------------
# fusing the two cues
# --------------------------------------------------------------------------

def _fuse(reading: VariantReading, sign_height: float) -> None:
    sides_variant = reading.sides_variant
    letter_variant = reading.letter_variant

    if sides_variant is None and letter_variant is None:
        reading.reason = "neither cue read"
        return
    if sides_variant is None:
        reading.variant = letter_variant
        reading.confidence = round(min(1.0, 0.35 + 2.0 * reading.letter_gap), 3)
        reading.reason = "letter only - no side count"
        return
    if letter_variant is None:
        reading.variant = sides_variant
        reading.confidence = round(min(1.0, 0.25 * reading.sides_margin), 3)
        reading.reason = "sides only - letter unreadable"
        return
    if sides_variant != letter_variant:
        # The cues cross over: measured, the silhouette is the more reliable of
        # the two below about sixty pixels and the letter above it, so a
        # disagreement is settled by which one is on its own ground - but only
        # when the winner is speaking clearly. A thin margin either way stays a
        # refusal, because a wrong bay costs more than a missed one.
        confident_sides = reading.sides_margin >= 2.0
        confident_letter = reading.letter_gap >= 0.15
        if sign_height < CUE_CROSSOVER and confident_sides and not confident_letter:
            reading.variant = sides_variant
            reading.confidence = round(min(0.6, 0.1 * reading.sides_margin), 3)
            reading.reason = f"sides over letter below {CUE_CROSSOVER} px"
            return
        if sign_height >= CUE_CROSSOVER and confident_letter and not confident_sides:
            reading.variant = letter_variant
            reading.confidence = round(min(0.6, 2.0 * reading.letter_gap), 3)
            reading.reason = f"letter over sides above {CUE_CROSSOVER} px"
            return
        reading.reason = (f"{reading.sides} sides says {sides_variant}, "
                          f"letter says {letter_variant}")
        return

    reading.variant = sides_variant
    reading.confidence = round(min(1.0, 0.3 + 0.12 * reading.sides_margin
                                   + 1.5 * reading.letter_gap), 3)
    reading.reason = "sides and letter agree"


# --------------------------------------------------------------------------
# plumbing
# --------------------------------------------------------------------------

def _to_work_height(crop: np.ndarray) -> np.ndarray:
    height = crop.shape[0]
    target = int(np.clip(height, _MIN_WORK_HEIGHT, _MAX_WORK_HEIGHT))
    if target == height:
        return crop
    scale = target / height
    interpolation = cv.INTER_CUBIC if scale > 1 else cv.INTER_AREA
    return cv.resize(crop, (max(4, int(round(crop.shape[1] * scale))), target),
                     interpolation=interpolation)
