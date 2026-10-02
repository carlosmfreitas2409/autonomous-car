"""Read a traffic light's state out of the housing crop.

The detector's job ends at "there is a housing here". Everything after is
ordinary image processing, which means it is unit-testable without a camera and
fixable without retraining.

Three things about the real venue photos drive every decision below, and all
three were measured on them rather than assumed:

  * The lit core is **pure white** - saturation around 5/255. There is no hue
    in it at all. All the colour survives in the halo, which spills well
    outside the housing and tints the wall behind it.
  * With yellow lit, the two unlit covers pick up that spill and read
    *brighter* than the housing. "Brightest region" is not "lit lamp".
  * With everything off, the three covers are grey-white discs that pass any
    pure brightness test, and the hall's warm stage lighting gives them genuine
    saturation. A colour-fraction gate calls that RED. Asking "is this
    emitting" instead of "is this colourful" is the fix.

Red hue measures 2-28 and yellow 29-31: they overlap, so hue cannot separate
them. Colour is only ever asked "warm or green". Which warm comes from *which
cell* is lit, and cell position comes from the cells this module finds rather
than from slicing the box into thirds - a 2.3:1 housing sliced into three bands
loses its middle one to rounding, and then every yellow reads as red.

`UNKNOWN` is a first-class answer. A guess about a traffic light is worse than
an admission.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import itertools

import cv2 as cv
import numpy as np


class LightState(Enum):
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    UNKNOWN = "UNKNOWN"


# Cell spacing over cell radius, measured off the housing in the venue photos:
# 89.5/41.5 at mid range, 110/51 up close, 41/19 far away - 2.15 every time.
# It is a property of the physical light, not a tuning knob.
SPACING_OVER_RADIUS = 2.15

# The housing edges relative to the cells, same measurement: the body reaches
# 1.48 radii either side of the cell axis, 1.53 above the top cell and 1.15
# below the bottom one. Used to turn found cells back into a housing box.
HOUSING_MARGIN_X = 1.48
HOUSING_MARGIN_TOP = 1.53
HOUSING_MARGIN_BOTTOM = 1.15

# Cells are found by cutting the crop at a ladder of levels. A blown-out lamp
# is a clean disc at the top of the ladder; an unlit cover under that lamp's
# spill is one near the bottom. Neither is found at a single level, which is
# why there is a ladder rather than a threshold.
#
# The ladder is in percentiles of the crop's own brightness, not in absolute
# grey: a light 5 m away in a dark hall and the same light filling the frame
# under stage lighting have nothing in common on an absolute scale, but in both
# the covers are the brightest fifth of the crop.
_LEVELS = (99.5, 99, 98, 96, 93, 90, 86, 81, 75, 68, 60, 52, 44, 36, 28)

# A round blob fills about pi/4 = 0.785 of its bounding box. The window is wide
# enough for perspective and for a cover clipped by the housing lip.
_BLOB_FILL = (0.60, 0.95)
_BLOB_ASPECT = (0.62, 1.60)

# Candidates kept per view. Cells are the biggest round things inside a
# housing; this only bounds the cost of the pair search, and raising it above
# ten changed nothing measurable on the venue photos.
_MAX_CANDIDATES = 10

# Everything is measured at this height. Small crops are upsampled because cell
# finding on a 40 px crop has nothing to cut; large ones are downsampled because
# every measurement below is in units of the cell radius anyway, and a housing
# 400 px tall carries no information a 200 px one does not.
_WORK_HEIGHT = 200

# Radii, in cell radii, of the regions each measurement uses.
_CORE = 0.72             # the cover/lamp face itself
_RING = (1.12, 1.45)     # housing body just outside a cover
_HALO = (1.00, 2.30)     # where the colour survives, core excluded

# Where the glow is looked for: a patch either side of each cell, on the strip
# of housing body between the cover's edge and the housing's. It has to be
# sideways - the covers are spaced 2.15 radii apart and are 2 radii across, so
# directly above and below a cell there is no body at all, only the next cover.
# It has to be the body rather than the wall further out, because the wall
# behind this light is a dark curtain at the top of the frame and a lit wooden
# floor at the bottom: measured on the wall, three of five reds pick the bottom
# cell, because the floor is brighter than the glow.
_BODY_OFFSET = 1.27
_BODY_HALF = 0.17

# What "something here is emitting" costs. Measured over the venue photos whose
# stack is found: the unlit housings reach 0.33 at most, and the lit ones start
# at 0.53. Half way between is 0.43, which is 24 %
# clear of both - not a slider, a separation that was measured once and would
# only move if the light itself changed.
EMISSION_FLOOR = 0.43

# How much room to leave around the housing box when cropping for this reader.
# The colour it needs is in the halo, and the halo is outside the housing - a
# crop tight to the box throws away the only hue in the frame. Swept over the
# venue photos: anywhere from 0.20 to 0.40 lands within one photo of the best,
# so this is a framing choice rather than a knob to turn.
CROP_MARGIN = 0.30

# When the glow has erased the housing entirely there are no cells to find, and
# the reader falls back on where the blown-out core sits inside the box the
# detector drew. Two things gate that fallback, both measured on the venue
# photographs:
#
#   * something in the box must actually be clipping the sensor. A lit lamp
#     reaches 253-254 at the 97th percentile in every one of the 30 lit photos;
#     an unlit housing reaches 183 at most, over 8. The gap is enormous and
#     this sits in the middle of it.
#   * the blown region must be *one lamp*, not the whole box. A cell is two
#     radii of a housing that is 6.98 radii tall, so a single lamp plus its
#     bloom stays well inside half the height; a bare lamp filling the box does
#     not, and must not be read as a traffic light.
_SATURATED = 230
_BLOWN_SPREAD = 0.45

# YELLOW is only ever answered when the glow is not red. Red versus yellow is
# otherwise decided by position alone, and a stack fitted one cell too high -
# a phantom cell in the red glow above the housing - turns the red lamp into
# the middle one. The policy lets a yellow roll through at crawl speed, so that
# is the misread that hurts. Measured over the whole crop, so it does not rest
# on the fit it is checking: every red lamp in every shoot has a glow at least
# 0.89 red rather than amber; yellows under ordinary light reach 0.73 at most.
# Yellows in front of a red-lit stage also read red, and become RED here - a
# needless stop, before a lamp that turns red next anyway.
_RED_GLOW = 0.80

# A pixel this bright carries no hue worth reading - it is the blown core, or
# the part of the halo that has also clipped. Measured: the core sits at
# saturation ~5 whatever colour is lit.
_BLOWN = 244


@dataclass
class CellStack:
    """Three lamp cells, found in the crop's own coordinates."""
    cx: float
    centres: tuple[float, float, float]
    radius: float
    contrast: tuple[float, float, float]
    # Cells whose centre falls outside the crop because the frame cut the
    # housing off. Never inferred from weak evidence - only from geometry.
    clipped: tuple[int, ...] = ()
    score: float = 0.0
    # True when the cells were taken from the crop's geometry rather than
    # found in it, because the lamp had washed the housing out.
    assumed: bool = False

    @property
    def found(self) -> int:
        return 3 - len(self.clipped)

    def housing_box(self) -> tuple[float, float, float, float]:
        r = self.radius
        return (
            self.cx - HOUSING_MARGIN_X * r,
            self.centres[0] - HOUSING_MARGIN_TOP * r,
            self.cx + HOUSING_MARGIN_X * r,
            self.centres[2] + HOUSING_MARGIN_BOTTOM * r,
        )


@dataclass
class LightReading:
    state: LightState = LightState.UNKNOWN
    confidence: float = 0.0
    cells: Optional[CellStack] = None
    lit_index: Optional[int] = None
    # How strongly each of the two independent cues speaks, 0..1. They are what
    # the decision weighs, and what the inspector prints.
    position_evidence: float = 0.0
    colour_evidence: float = 0.0
    warm_votes: float = 0.0
    green_votes: float = 0.0
    emission: tuple[float, ...] = ()
    reason: str = ""
    # Debug imagery for tools/inspect_signs.py. Never used for the decision.
    debug: dict = field(default_factory=dict)

    @property
    def verified(self) -> bool:
        """Did this crop verify as a three-cell stack at all?

        This is the second gate, independent of the detector: a stage lamp, a
        phone screen and a standing person all fail it. Aspect ratio provably
        cannot do this job - real housings measure h/w 2.2-2.45, box jitter
        swings that to 1.15-3.22, and standing people measure 2.7-4.8.
        """
        return self.cells is not None


def read_light_state(crop: np.ndarray) -> LightReading:
    """Read RED / YELLOW / GREEN / UNKNOWN from a traffic-light crop.

    The crop should be the housing with some margin around it - the colour
    lives in the halo, which is outside the housing.
    """
    if crop is None or crop.size == 0 or min(crop.shape[:2]) < 8:
        return LightReading(reason="crop too small")

    work, scale = _to_work_height(crop)
    reading = _read(work, scale)
    # Only a yellow whose own halo voted warm: a halo with no colour says
    # nothing about the lamp, and the crop's glow can then be a red-lit stage
    # behind a green lamp the fit got wrong.
    warm_halo = reading.warm_votes > reading.green_votes and reading.colour_evidence > 0.05
    if reading.state is LightState.YELLOW and warm_halo:
        share = _red_glow_share(work)
        if share >= _RED_GLOW:
            reading.state = LightState.RED
            reading.reason = f"glow is {share:.0%} red, not amber - {reading.reason}"
    return reading


def _read(work: np.ndarray, scale: float) -> LightReading:
    gray = _blur(cv.cvtColor(work, cv.COLOR_BGR2GRAY))

    stack = find_cell_stack(work)
    if stack is None:
        return _read_by_blown_core(work, gray, scale)

    emission = _emission_per_cell(gray, stack)
    reading = LightReading(
        cells=_rescale(stack, scale),
        emission=tuple(round(float(e), 3) for e in emission),
    )

    lit, position_evidence = _lit_cell(emission)
    reading.position_evidence = position_evidence
    if lit is None:
        reading.reason = "no cell is emitting"
        reading.debug = _debug_masks(work, stack, None)
        return reading
    reading.lit_index = lit

    warm, green = _halo_votes(work, stack, lit)
    total = warm + green
    reading.warm_votes, reading.green_votes = warm, green
    reading.colour_evidence = abs(warm - green) / total if total > 0 else 0.0
    colour_family = None if total <= 0 else ("warm" if warm > green else "green")

    _decide(reading, lit, colour_family)
    reading.debug = _debug_masks(work, stack, lit)
    return reading


def _read_by_blown_core(work: np.ndarray, gray: np.ndarray, scale: float) -> LightReading:
    """Last resort: read the lamp from where its blown core sits in the box.

    A lamp bright enough to erase its own housing leaves nothing for the cell
    finder to work with - measured on the yellow frames of the approach clip,
    every candidate disc is the flooded region itself and not one of the three
    hypotheses clears the contrast test, because the premise that a cover is
    lighter than the body around it has stopped being true.

    What survives is position. The crop is the housing box grown by
    CROP_MARGIN, so the three cells sit at fixed fractions of its height -
    0.22, 0.53 and 0.84, straight out of the housing geometry above. Measured
    over the clip, the blown core's centroid lands at 0.18-0.26 for red,
    0.49-0.51 for yellow and 0.78-0.83 for green: three tight, well separated
    clusters.

    This is emphatically *not* the old "slice the box into three bands" bug,
    which rounded a 2.3:1 housing into two and lost the middle lamp entirely.
    Nothing is rounded: a continuous centroid is compared against three known
    positions, and an answer is only given when it is clearly nearest to one of
    them.
    """
    height, width = gray.shape
    inset = CROP_MARGIN / (1 + 2 * CROP_MARGIN)
    top, bottom = inset * height, (1 - inset) * height
    box_height = max(1.0, bottom - top)

    inner = gray[int(top):int(bottom), int(inset * width):int((1 - inset) * width)]
    if inner.size < 64:
        return LightReading(reason="no three-cell stack")

    level = float(np.percentile(inner, 97))
    if level < _SATURATED:
        return LightReading(reason="no three-cell stack, and nothing in the box is lit")

    rows, _ = np.nonzero(gray >= level)
    if rows.size < 20:
        return LightReading(reason="no three-cell stack")
    centre = (float(rows.mean()) - top) / box_height
    spread = float(np.percentile(rows, 90) - np.percentile(rows, 10)) / box_height
    if spread > _BLOWN_SPREAD:
        return LightReading(
            reason=f"no three-cell stack, and the bright region spans {spread:.0%} "
                   f"of the box - one lamp does not")

    # This path may only ever answer YELLOW, and that restriction is the whole
    # reason it is safe to have.
    #
    # Yellow is the one state the main path structurally cannot read: it is the
    # middle lamp, so its flood erases the covers on *both* sides and no three
    # cells survive. Red and green flood one neighbour and are read correctly
    # 12/12 and 9/9. So the gap is yellow-shaped, and a fallback that can only
    # say yellow fills exactly it.
    #
    # It also bounds the damage. Position alone is not reliable here - on the
    # two closest venue photos the housing is cut off by the frame, the
    # fractions below mean nothing, and the blown core lands at 0.32, nearer
    # red's 0.22 than yellow's 0.53. Allowed to answer freely this path calls
    # both of them RED, confidently, turning two safe refusals into two wrong
    # answers. Restricted to yellow it refuses them instead, and it can never
    # produce GREEN. YELLOW on a red lamp is not harmless - the policy lets a
    # yellow roll through at crawl speed - which is what the red-glow check in
    # read_light_state is for; it covers this path too.
    offsets = _cell_offsets()
    lit = 1
    spacing = offsets[1] - offsets[0]
    distance = abs(centre - offsets[lit])
    if distance > 0.4 * spacing:
        return LightReading(
            reason=f"no three-cell stack, and the bright core sits at {centre:.2f} "
                   f"of the box, not the middle lamp's {offsets[lit]:.2f}")

    stack = _stack_from_box(width, height)
    reading = LightReading(cells=_rescale(stack, scale), lit_index=lit)
    reading.position_evidence = float(np.clip(1.0 - distance / (0.4 * spacing), 0.0, 1.0))

    warm, green = _halo_votes(work, stack, lit)
    total = warm + green
    reading.warm_votes, reading.green_votes = warm, green
    reading.colour_evidence = abs(warm - green) / total if total > 0 else 0.0
    family = None if total <= 0 else ("warm" if warm > green else "green")

    _decide(reading, lit, family)
    # Half credit: this path has one cue where the main one has two.
    reading.confidence = round(0.5 * reading.confidence, 3)
    reading.reason = f"housing washed out - {reading.reason}, core at {centre:.2f}"
    reading.debug = _debug_masks(work, stack, lit)
    return reading


def _cell_offsets() -> tuple[float, float, float]:
    """Where the three cells sit down the housing, as fractions of its height.

    Derived from the housing geometry, not measured separately: the body is
    HOUSING_MARGIN_TOP + 2 spacings + HOUSING_MARGIN_BOTTOM radii tall.
    """
    total = HOUSING_MARGIN_TOP + HOUSING_MARGIN_BOTTOM + 2 * SPACING_OVER_RADIUS
    return tuple((HOUSING_MARGIN_TOP + i * SPACING_OVER_RADIUS) / total for i in range(3))


def _stack_from_box(width: int, height: int) -> CellStack:
    """The stack the crop's own geometry implies, when the image will not show
    one. Flagged by `assumed` so nothing downstream mistakes it for a fit."""
    inset = CROP_MARGIN / (1 + 2 * CROP_MARGIN)
    top, bottom = inset * height, (1 - inset) * height
    box_height = bottom - top
    total = HOUSING_MARGIN_TOP + HOUSING_MARGIN_BOTTOM + 2 * SPACING_OVER_RADIUS
    radius = box_height / total
    centres = tuple(top + offset * box_height for offset in _cell_offsets())
    return CellStack(width / 2.0, centres, radius, (0.0, 0.0, 0.0), assumed=True)


# --------------------------------------------------------------------------
# 1. structural verification: is this a three-cell stack at all?
# --------------------------------------------------------------------------

def find_cell_stack(image: np.ndarray, min_radius: float = 3.0,
                    max_radius: float = 1e9) -> Optional[CellStack]:
    """Fit three round, co-linear, evenly spaced cells to a crop.

    Hypotheses come from *pairs* of candidate discs rather than triples,
    because the cell that matters most is the one hardest to find: a lit lamp
    bleeds into its neighbours and stops being a blob of its own. Any two cells
    plus the known spacing-to-radius ratio pin the third down, and the fit is
    then scored against the image rather than trusted.

    Candidates are proposed from three views of the crop and scored on one.
    Plain brightness loses a cover that a lit lamp has flooded; the per-pixel
    minimum of the three channels is almost untouched by a blown white core and
    almost erased by coloured glow, so a flooded cover comes back as a disc
    there; and CLAHE flattens the glow's gradient, which recovers the rest.
    Measured over the 38 venue photos, at three different crop margins: grey
    alone finds the stack in 29, grey and the minimum in 34, all three in 36 -
    and it is never the same ones that fail. Scoring stays on plain brightness,
    which is what "a cover is lighter than the body it sits in" is a statement
    about.
    """
    if image is None or image.size == 0:
        return None
    gray = _blur(cv.cvtColor(image, cv.COLOR_BGR2GRAY) if image.ndim == 3 else image)
    candidates = _cell_candidates(gray)
    candidates += _cell_candidates(_blur(_clahe().apply(gray)))
    if image.ndim == 3:
        candidates += _cell_candidates(_blur(image.min(axis=2)))

    sums = _Sums(gray)
    best: Optional[CellStack] = None
    for cx, centres, radius in _hypotheses(candidates):
        if not (min_radius <= radius <= max_radius):
            continue
        measured = _score_cells(sums, cx, centres, radius)
        if measured is None:
            continue
        contrast, clipped = measured
        present = [c for i, c in enumerate(contrast) if i not in clipped]
        if len(present) < 2 or min(present) <= 0.02:
            continue
        # Scale-free on purpose: a stack is judged on how cell-like its cells
        # are, not on how big it is. A stack the frame cut in half is worth
        # having but is never preferred to one that is all there.
        score = (min(present) + float(np.mean(present))) * (0.45 if clipped else 1.0)
        if best is None or score > best.score:
            best = CellStack(cx, tuple(centres), radius, tuple(contrast),
                             tuple(clipped), score)
    return best


def _blur(single_channel: np.ndarray) -> np.ndarray:
    return cv.GaussianBlur(single_channel, (5, 5), 0)


def _clahe():
    return cv.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))


def _cell_candidates(gray: np.ndarray) -> list[tuple[float, float, float]]:
    """Discs the cells might be, from two views of the same threshold ladder.

    Connected components find a cell that stands alone. They cannot find one
    that has fused with its neighbour, which is precisely what a lit lamp does
    to the covers above and below it - at every level where the covers appear,
    all three are one blob. The distance transform of that same blob peaks once
    per disc, so it splits them back apart. Neither view alone covers the
    range; together they do.
    """
    found: list[tuple[float, float, float]] = []
    area = float(gray.size)
    for level in np.unique(np.round(np.percentile(gray, [100 - p for p in _LEVELS]))):
        mask = (gray >= level).astype(np.uint8)
        if not 0.01 <= float(mask.sum()) / area <= 0.70:
            continue
        _components(mask, found)
        _distance_peaks(mask, found)

    # The same cover appears at several levels. Keep the largest of each group.
    kept: list[tuple[float, float, float]] = []
    for blob in sorted(found, key=lambda b: -b[2]):
        if any(np.hypot(blob[0] - k[0], blob[1] - k[1]) < 0.6 * max(blob[2], k[2])
               and 0.6 <= blob[2] / k[2] <= 1.7 for k in kept):
            continue
        kept.append(blob)
        if len(kept) >= _MAX_CANDIDATES:
            break
    return kept


def _components(mask: np.ndarray, found: list) -> None:
    count, _, stats, centroids = cv.connectedComponentsWithStats(mask, 8)
    for i in range(1, count):
        _, _, w, h, area = stats[i]
        if area < 16 or w < 4 or h < 4:
            continue
        if not _BLOB_FILL[0] <= area / float(w * h) <= _BLOB_FILL[1]:
            continue
        if not _BLOB_ASPECT[0] <= w / float(h) <= _BLOB_ASPECT[1]:
            continue
        found.append((float(centroids[i][0]), float(centroids[i][1]),
                      float(np.sqrt(area / np.pi))))


def _distance_peaks(mask: np.ndarray, found: list) -> None:
    distance = cv.distanceTransform(mask, cv.DIST_L2, 3, dstType=cv.CV_32F)
    peak = float(distance.max())
    if peak < 3:
        return
    # One local maximum per disc: suppress over a window a little wider than
    # the largest disc present, so two touching covers keep one peak each.
    window = min(61, int(max(3, round(1.2 * peak))) | 1)
    local_max = cv.dilate(distance, np.ones((window, window), np.uint8))
    ys, xs = np.nonzero((distance >= local_max - 1e-3) & (distance > 0.35 * peak))
    for x, y in zip(xs, ys):
        found.append((float(x), float(y), float(distance[y, x])))


def _hypotheses(blobs: list[tuple[float, float, float]]):
    """Every three-cell stack two candidate discs could be part of.

    The windows are wide because a candidate's own radius is the least
    trustworthy thing about it - bloom inflates it, a high cut shrinks it. The
    radius that gets used comes from the *gap*, which neither affects, and the
    fit is verified against the image afterwards either way.
    """
    for upper, lower in itertools.combinations(sorted(blobs, key=lambda b: b[1]), 2):
        gap = lower[1] - upper[1]
        mean_r = 0.5 * (upper[2] + lower[2])
        if gap < 6 or abs(upper[0] - lower[0]) > 0.8 * mean_r:
            continue
        cx = 0.5 * (upper[0] + lower[0])
        if 1.2 * mean_r <= gap <= 4.0 * mean_r:      # adjacent cells
            radius = gap / SPACING_OVER_RADIUS
            yield cx, [upper[1] - gap, upper[1], lower[1]], radius
            yield cx, [upper[1], lower[1], lower[1] + gap], radius
        if 2.6 * mean_r <= gap <= 8.0 * mean_r:      # the outer two
            radius = gap / (2 * SPACING_OVER_RADIUS)
            yield cx, [upper[1], 0.5 * (upper[1] + lower[1]), lower[1]], radius


class _Sums:
    """Integral image, so a region mean costs four lookups.

    The pair search scores a few hundred hypotheses per crop and every one asks
    for the mean of a cell face and of the housing around it. Masked circles are
    the honest shape and far too slow at that rate; concentric squares put the
    same pixels in the same two groups - the square inside a cell's face sits
    entirely on the cover, and the band outside it sits on the body.
    """

    def __init__(self, gray: np.ndarray):
        self.shape = gray.shape
        self.integral = cv.integral(gray.astype(np.float64))

    def _sum(self, cx: float, cy: float, half: float) -> tuple[float, int]:
        height, width = self.shape
        x0 = int(np.clip(round(cx - half), 0, width))
        x1 = int(np.clip(round(cx + half) + 1, 0, width))
        y0 = int(np.clip(round(cy - half), 0, height))
        y1 = int(np.clip(round(cy + half) + 1, 0, height))
        if x1 <= x0 or y1 <= y0:
            return 0.0, 0
        total = (self.integral[y1, x1] - self.integral[y0, x1]
                 - self.integral[y1, x0] + self.integral[y0, x0])
        return float(total), (x1 - x0) * (y1 - y0)

    def block(self, cx: float, cy: float, half: float) -> Optional[float]:
        total, count = self._sum(cx, cy, half)
        return None if count < 4 else total / count

    def band(self, cx: float, cy: float, inner: float, outer: float) -> Optional[float]:
        outer_sum, outer_n = self._sum(cx, cy, outer)
        inner_sum, inner_n = self._sum(cx, cy, inner)
        count = outer_n - inner_n
        return None if count < 8 else (outer_sum - inner_sum) / count


def _score_cells(sums: "_Sums", cx, centres, radius):
    """Contrast of each cell against the housing body just outside it.

    A cover is brighter than the black body it sits in, lit or not. A cell
    whose centre falls outside the image is reported as clipped rather than as
    absent - the frame cutting the housing off is not evidence against the
    housing. Anything else with no contrast is simply not a cell.
    """
    height, width = sums.shape
    contrast, clipped = [], []
    for index, cy in enumerate(centres):
        core = body = None
        if 0 <= cx < width and 0 <= cy < height:
            core = sums.block(cx, cy, _CORE * radius)
            body = sums.band(cx, cy, _RING[0] * radius, _RING[1] * radius)
        if core is None or body is None:
            clipped.append(index)
            contrast.append(0.0)
            continue
        contrast.append((core - body) / (core + body + 1e-6))
    if len(clipped) > 1:
        return None
    return contrast, clipped


# --------------------------------------------------------------------------
# 2-3. which cell is emitting - emission, not brightness
# --------------------------------------------------------------------------

def _emission_per_cell(gray: np.ndarray, stack: CellStack) -> np.ndarray:
    """How much each cell looks like a source rather than a reflector.

    This is the M2 fix, and it is a change of question rather than a change of
    threshold. A colour-fraction gate asks "is this thing colourful", and the
    venue's warm stage lighting answers yes for a grey plastic cover, which is
    how five of six unlit housings came to be called RED. The physically right
    question is "is this thing a source", and a source does two things a
    reflector cannot:

      * **glow** - it lights up the housing either side of itself. Measured
        against the darkest of the three cells' own surroundings, so it is a
        comparison inside one crop and a hall that is bright, dark or lit by
        coloured lamps moves all three together and cancels out.
      * **blaze** - its face is far brighter than its neighbours'. With
        everything off the three covers are the same grey; with one lamp lit
        its face clips to white while the others do not.

    Neither cue is enough alone. Red at mid range glows barely more than an
    unlit housing in a bright hall, and a distant green lamp is dim enough that
    its face barely blazes. Their sum separates every photo we have.
    """
    sums = _Sums(gray)
    radius = stack.radius
    body, face = [], []
    for index, cy in enumerate(stack.centres):
        if index in stack.clipped:
            body.append(None)
            face.append(None)
            continue
        beside = [sums.block(stack.cx + sign * _BODY_OFFSET * radius, cy, _BODY_HALF * radius)
                  for sign in (-1, 1)]
        beside = [v for v in beside if v is not None]
        body.append(float(np.mean(beside)) if beside else None)
        face.append(sums.block(stack.cx, cy, _CORE * radius))

    known_body = [v for v in body if v is not None]
    known_face = [v for v in face if v is not None]
    if len(known_body) < 2 or len(known_face) < 2:
        return np.full(3, -1.0, dtype=np.float32)
    unlit_body, brightest_face = min(known_body), max(known_face)

    values = []
    for index in range(3):
        if body[index] is None or face[index] is None:
            values.append(-1.0)
            continue
        others = [v for i, v in enumerate(face) if i != index and v is not None]
        glow = (body[index] - unlit_body) / max(8.0, brightest_face - unlit_body)
        blaze = (face[index] - max(others)) / max(8.0, face[index]) if others else 0.0
        values.append(float(glow + blaze))
    return np.array(values, dtype=np.float32)


def _lit_cell(emission: np.ndarray) -> tuple[Optional[int], float]:
    """The emitting cell, and how clearly it beats the others.

    If nothing clears the floor there is no answer to give: every lamp is off,
    or the housing is washed out past reading. Saying so is the whole point -
    an unlit light called RED stops the car in front of a green one.
    """
    order = np.argsort(emission)[::-1]
    best, runner_up = int(order[0]), int(order[1])
    if emission[best] < EMISSION_FLOOR:
        return None, 0.0
    margin = (emission[best] - emission[runner_up]) / abs(emission[best])
    return best, float(np.clip(margin, 0.0, 1.0))


# --------------------------------------------------------------------------
# 4-5. colour from the halo, and only ever "warm or green"
# --------------------------------------------------------------------------

def _halo_votes(bgr: np.ndarray, stack: CellStack, lit: int) -> tuple[float, float]:
    """Warm and green votes from the ring around the lit lamp.

    Votes, never an average: red wraps around the 0/180 boundary, so averaging
    a red halo gives cyan. Each pixel's vote is weighted by its own saturation,
    which means a washed-out halo contributes almost nothing instead of
    contributing noise - and that removes the need for a saturation floor,
    which is the constant that is impossible to set once yellow is in the room
    (yellow's ring peaks near S=98 where red reaches 177).
    """
    mask = _halo_mask(bgr.shape[:2], stack, lit)
    if mask is None or mask.sum() < 12:
        return 0.0, 0.0

    hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)
    hue = hsv[:, :, 0][mask].astype(np.int32)
    sat = hsv[:, :, 1][mask].astype(np.float32)
    val = hsv[:, :, 2][mask].astype(np.float32)

    usable = val < _BLOWN
    if usable.sum() < 12:
        return 0.0, 0.0
    hue, sat = hue[usable], sat[usable]

    weight = sat / 255.0
    warm = ((hue <= 40) | (hue >= 165))
    green = (hue >= 45) & (hue <= 95)
    return float(weight[warm].sum()), float(weight[green].sum())


def _red_glow_share(bgr: np.ndarray) -> float:
    """How much of the crop's coloured glow is red rather than amber, 0..1.

    The whole crop, not the halo of one cell, because the cell is what it
    checks. Only pixels with real colour count - saturation 90 and up, neither
    dark nor blown - weighted by saturation as in _halo_votes. A crop with no
    colour at all says 0: no evidence against yellow.
    """
    hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].astype(np.int32)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2]
    coloured = (sat >= 90) & (val >= 60) & (val < _BLOWN)
    weight = sat / 255.0
    red = float(weight[coloured & ((hue <= 7) | (hue >= 170))].sum())
    amber = float(weight[coloured & (hue >= 11) & (hue <= 35)].sum())
    return red / (red + amber) if red + amber > 0 else 0.0


def _halo_mask(shape, stack: CellStack, lit: int) -> Optional[np.ndarray]:
    height, width = shape
    cy = stack.centres[lit]
    r = stack.radius
    yy, xx = np.mgrid[0:height, 0:width]
    distance = np.hypot(xx - stack.cx, yy - cy)
    return (distance >= _HALO[0] * r) & (distance <= _HALO[1] * r)


# --------------------------------------------------------------------------
# 6. decision
# --------------------------------------------------------------------------

_BY_POSITION = (LightState.RED, LightState.YELLOW, LightState.GREEN)


def _decide(reading: LightReading, lit: int, colour_family: Optional[str]) -> None:
    """Weigh position against colour, and refuse when they disagree.

    Neither cue is trusted on its own: a washed-out lamp is judged mostly on
    position, a loose or clipped box mostly on colour. When they point at
    different answers and neither dominates, the answer is UNKNOWN.
    """
    by_position = _BY_POSITION[lit]
    position_weight = reading.position_evidence
    colour_weight = reading.colour_evidence

    if colour_family is None or colour_weight <= 0.05:
        reading.state = by_position
        reading.confidence = round(0.55 * position_weight, 3)
        reading.reason = "position only - halo carries no colour"
        return

    agrees = (colour_family == "green") == (by_position is LightState.GREEN)
    if agrees:
        reading.state = by_position
        reading.confidence = round(min(1.0, 0.45 + 0.3 * position_weight + 0.25 * colour_weight), 3)
        reading.reason = "position and colour agree"
        return

    if position_weight > 2 * colour_weight:
        reading.state = by_position
        reading.confidence = round(0.4 * position_weight, 3)
        reading.reason = f"position over colour ({colour_family} halo)"
        return

    if colour_weight > 2 * position_weight and colour_family == "green":
        reading.state = LightState.GREEN
        reading.confidence = round(0.4 * colour_weight, 3)
        reading.reason = "green halo over position"
        return

    # A warm halo on the green cell cannot name itself red or yellow, and a
    # green halo on a warm cell is a contradiction. Both are UNKNOWN.
    reading.state = LightState.UNKNOWN
    reading.confidence = 0.0
    reading.reason = f"position says {by_position.value}, halo says {colour_family}"


# --------------------------------------------------------------------------
# plumbing
# --------------------------------------------------------------------------

def _to_work_height(crop: np.ndarray) -> tuple[np.ndarray, float]:
    """Put every crop on the same pixel scale, whatever range it came from."""
    scale = _WORK_HEIGHT / crop.shape[0]
    if 0.85 <= scale <= 1.15:
        return crop, 1.0
    interpolation = cv.INTER_CUBIC if scale > 1 else cv.INTER_AREA
    resized = cv.resize(crop, (max(1, int(round(crop.shape[1] * scale))), _WORK_HEIGHT),
                        interpolation=interpolation)
    return resized, scale


def _rescale(stack: CellStack, scale: float) -> CellStack:
    if scale == 1.0:
        return stack
    return CellStack(
        stack.cx / scale,
        tuple(c / scale for c in stack.centres),
        stack.radius / scale,
        stack.contrast,
        stack.clipped,
        stack.score,
        stack.assumed,
    )


def _debug_masks(work: np.ndarray, stack: CellStack, lit: Optional[int]) -> dict:
    """Two small images the inspector shows: what was read, and from where."""
    gray = cv.cvtColor(work, cv.COLOR_BGR2GRAY)
    core = np.zeros(gray.shape, np.uint8)
    for index, cy in enumerate(stack.centres):
        if index in stack.clipped:
            continue
        cv.circle(core, (int(round(stack.cx)), int(round(cy))),
                  max(1, int(round(_CORE * stack.radius))),
                  255 if index == lit else 90, -1)
    halo = np.zeros(gray.shape, np.uint8)
    if lit is not None:
        mask = _halo_mask(gray.shape, stack, lit)
        halo[mask] = 255
    return {"work": work, "core_mask": core, "halo_mask": halo}
