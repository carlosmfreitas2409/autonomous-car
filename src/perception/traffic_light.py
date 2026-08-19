"""Traffic light state read from the lit lamp, with no learned model involved.

The detector answers "is there a traffic light, and where" — a question about
shape and context, which is what a neural net is good at. This module answers
"which lamp is lit", which is a question about geometry and colour. We can get
that exactly right with arithmetic, test it against synthetic images, and debug
it without retraining anything.

Two signals decide the state, and each is weighted by its *own* measured
reliability rather than by a tuning knob:

  Position  Top/middle/bottom maps to red/yellow/green. Immune to white
            balance, exposure, and to the LED core clipping to white. Trusted
            in proportion to how strongly the crop's aspect ratio implies a
            real multi-lamp housing.

  Hue       The colour of the lit pixels, sampled only where saturation is
            high. This is the detail that matters: a bright LED blows its
            centre out to near-white (S ~ 0, V = 255), so the hue has to come
            from the saturated halo around the core, never from the middle.
            Trusted in proportion to how much saturated colour there actually
            is.

Red and yellow sit next to each other in hue and are the classic misread.
Position separates them cleanly, which is why position is a first-class signal
here rather than a fallback.

When the two signals disagree, or when no lamp wins by a clear margin, the
answer is UNKNOWN. That is a real answer that the policy handles explicitly,
not a failure.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import cv2 as cv
import numpy as np


class LightState(Enum):
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"
    UNKNOWN = "unknown"


# Fraction of each side trimmed off the detector's box before anything is
# measured. The housing edge and whatever sits behind the pole are the single
# biggest source of wrong answers, and they live at the border.
_HOUSING_INSET = 0.08

# Below this the crop cannot be split into distinguishable lamps at all.
_MIN_CROP_PX = 8

# How close to the crop's own brightest pixel a pixel must be to count as part
# of the emitter. Scaling off the crop max is what makes this track exposure
# automatically instead of needing a brightness slider.
#
# Deliberately tight. On this project's light, a lit lamp spills enough light
# inside the housing to make the neighbouring cover glow almost as brightly as
# itself, and a loose threshold lets that glow win the cell vote — a lit middle
# lamp then reads as the top one, turning yellow into red. Only the emitter
# core actually saturates the sensor, so restricting to it separates the lamp
# that is lit from the one merely lit *upon*.
_LIT_V_RATIO = 0.92

# Absolute floor on "lit". Without it, a housing with every lamp dark still
# produces a brightest region and would elect a phantom winner.
_MIN_LIT_V = 60

# The winning lamp must cover at least this fraction of its own cell, or the
# crop is not showing a lamp at all.
_MIN_LIT_FRACTION = 0.02

# How far the winning cell must beat the runner-up to be believed completely.
# Not a pass/fail gate: a lit lamp makes the neighbouring cover glow almost as
# brightly, so on the real camera the margin is often slim — measured at 1.17
# for yellow. A slim margin should weaken the position signal, not discard it.
_LIT_MARGIN = 1.6

# The coloured ring lives between this fraction of peak brightness and the
# clipped core. Hue is read here and nowhere else.
#
# This is the whole trick. On the real camera a lit lamp is so much brighter
# than the scene that its centre clips to pure white — measured saturation in
# the core is a median of 5 out of 255, i.e. no colour at all. All of the
# colour survives in the ring just outside it. Reading hue from the lit region
# itself returned nothing for yellow and gave up entirely.
_HALO_V_RATIO = 0.70

# Saturation floor for a pixel to get a hue vote.
#
# Low, because yellow is the worst case: red and green each sit near one corner
# of the RGB cube, but yellow has two channels already high, so it desaturates
# toward white far sooner. Measured on the real camera, the yellow lamp's ring
# peaks at S=98 while red reaches 177 and green 138 — a threshold set for red
# silently excludes yellow completely.
_MIN_HUE_SATURATION = 50

# Hue needs this many saturated pixels before it is allowed to weigh in.
_MIN_HUE_PIXELS = 8

# A lit lamp must carry at least this much colour relative to the size of the
# bright region, or the bright region is not a lamp.
#
# This is what separates a lit lamp from an unlit one. An unlit cover is
# reflective grey plastic: it can be the brightest thing in the crop and pass
# every brightness test, but it has no colour. Measured on the real housing,
# unlit covers carry 12 to 53 coloured pixels against a bright region of 5,000
# to 8,000 — a fraction of 0.002 — while a lit lamp measures 0.2 to 1.0.
_MIN_COLOUR_FRACTION = 0.02

# How tall a full three-aspect housing measures relative to its width.
#
# Not the textbook 3:1. Measured on this project's light it is 2.0 to 2.4,
# because the lamps nearly fill the housing width and leave little border. An
# earlier version inferred the lamp count by rounding this ratio, which turned
# a 2.3:1 three-lamp housing into a two-lamp one — and a two-lamp layout has no
# middle position, so every yellow was reported as red. The count is now fixed
# at three, which is what a traffic light is, and the ratio is used only to
# judge whether the crop plausibly holds a whole housing.
_HOUSING_RATIO_RANGE = (1.8, 3.4)

# How far outside that range the ratio may drift before position stops being
# believable. A box cropped down to a single lamp measures near 1:1 and lands
# here, which is what hands the decision to hue.
_LAYOUT_TOLERANCE = 1.2

# Normalised score gap the winner needs over the runner-up. Set so that a
# straight position-vs-hue disagreement lands on UNKNOWN rather than guessing.
_DECISION_MARGIN = 0.15

# When the lamp cells are inconclusive, colour alone may still answer — but
# only if this much of the lit area agrees on one band.
_HUE_ONLY_DOMINANCE = 0.6

# Position and hue each contribute at most 1.0. Confidence is scored against
# this fixed ceiling rather than against the weight that happened to be
# available, so "both signals agree, loudly" outranks "both signals agree, but
# one of them barely spoke" — which is the difference between a clean lamp and
# a blown-out one. The decision margin still normalises by the available
# weight, since a weak signal must still be allowed to decide.
_MAX_SIGNAL_WEIGHT = 2.0

# OpenCV hue bands (0..179). Hue is asked only what it can actually answer.
#
# Measured on this project's own lamps, red reads anywhere from hue 2 to 28 and
# yellow from 29 to 31 — overlapping ranges that no threshold can separate. A
# bright red LED desaturates toward orange as it clips, and that drags it into
# yellow territory. Splitting them by hue produced confident *wrong* answers.
#
# So hue distinguishes warm from green, which it does with enormous margin
# (green measures 76–89), and position separates red from yellow, which it does
# perfectly. Each signal is used only where it is trustworthy.
_WARM_BANDS = ((0, 40), (165, 179))
_GREEN_BANDS = ((41, 110),)

# What each lamp position means, ordered top-to-bottom on a vertical housing
# or left-to-right on a horizontal one.
_LAMP_ORDER = (LightState.RED, LightState.YELLOW, LightState.GREEN)
_LAMP_COUNT = len(_LAMP_ORDER)

_COLOURED_STATES = (LightState.RED, LightState.YELLOW, LightState.GREEN)


@dataclass
class LightReading:
    state: LightState
    confidence: float
    lamp_index: Optional[int]
    n_lamps: int


def _unknown(n_lamps: int = 0) -> LightReading:
    return LightReading(state=LightState.UNKNOWN, confidence=0.0, lamp_index=None, n_lamps=n_lamps)


def read_light_state(crop: np.ndarray) -> LightReading:
    """Classify a traffic light housing crop (BGR) into a lamp state."""
    if crop is None or crop.size == 0 or crop.ndim != 3:
        return _unknown()

    inner = _inset(crop)
    height, width = inner.shape[:2]
    if height < _MIN_CROP_PX or width < _MIN_CROP_PX:
        return _unknown()

    vertical = height >= width
    long_side, short_side = (height, width) if vertical else (width, height)
    ratio = long_side / max(short_side, 1)
    n_lamps = _LAMP_COUNT

    hsv = cv.cvtColor(inner, cv.COLOR_BGR2HSV)
    hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    peak = float(value.max())
    # The emitter: bright enough to have clipped. This locates the lamp.
    core = value >= max(_MIN_LIT_V, _LIT_V_RATIO * peak)
    if not core.any():
        return _unknown(n_lamps)

    # Everywhere bright enough to belong to the lamp *and* still carrying
    # colour. On the real camera that is the ring, because the core has clipped
    # to white and fails the saturation test on its own; on a dimmer or better
    # exposed lamp the core qualifies too. No upper bound on brightness is
    # needed — saturation already excludes whatever has washed out.
    halo = (value >= _HALO_V_RATIO * peak) & (saturation >= _MIN_HUE_SATURATION)

    # No colour anywhere means nothing is emitting: this is an unlit housing
    # whose reflective covers happen to be the brightest thing in the crop.
    # Deciding from position alone here would report a state for every dark
    # light on the track.
    if np.count_nonzero(halo) < _MIN_COLOUR_FRACTION * np.count_nonzero(core):
        return _unknown(n_lamps)

    counts, slices = _lamp_cell_counts(core, n_lamps, vertical)
    winner = int(np.argmax(counts))
    runner_up = float(np.sort(counts)[-2])
    cell_area = max(core[slices[winner]].size, 1)

    if counts[winner] / cell_area < _MIN_LIT_FRACTION:
        return _hue_only(hue, halo, n_lamps)

    # Position is trusted in proportion to two things: how much the crop looks
    # like a whole housing, and how cleanly the winning cell beat the next one.
    separation = counts[winner] / max(runner_up, 1.0)
    w_pos = _layout_confidence(ratio) * _clamp01((separation - 1.0) / (_LIT_MARGIN - 1.0))
    position_state = _LAMP_ORDER[winner]

    # Prefer the winning lamp's own ring; fall back to the whole crop's when
    # the lamp is blown out so hard that almost nothing survives locally.
    cell_halo = np.zeros_like(halo)
    cell_halo[slices[winner]] = halo[slices[winner]]
    if np.count_nonzero(cell_halo) < _MIN_HUE_PIXELS:
        cell_halo = halo
    w_hue, hue_shares = _hue_shares(hue, cell_halo, counts[winner])

    total_weight = w_pos + w_hue
    if total_weight <= 0.0:
        return _unknown(n_lamps)

    scores = {
        state: w_pos * (1.0 if state is position_state else 0.0) + w_hue * hue_shares.get(state, 0.0)
        for state in _COLOURED_STATES
    }
    ranked = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    best_state, best_score = ranked[0]
    second_score = ranked[1][1]

    if (best_score - second_score) / total_weight < _DECISION_MARGIN:
        return _unknown(n_lamps)

    return LightReading(
        state=best_state,
        confidence=_clamp01(best_score / _MAX_SIGNAL_WEIGHT),
        lamp_index=winner,
        n_lamps=n_lamps,
    )


def _layout_confidence(ratio: float) -> float:
    """How much the crop's proportions look like a whole three-lamp housing."""
    low, high = _HOUSING_RATIO_RANGE
    if low <= ratio <= high:
        return 1.0
    gap = low - ratio if ratio < low else ratio - high
    return _clamp01(1.0 - gap / _LAYOUT_TOLERANCE)


def _hue_only(hue: np.ndarray, halo: np.ndarray, n_lamps: int) -> LightReading:
    """Decide from colour alone, for when the lamp geometry is unreadable."""
    weight, shares = _hue_shares(hue, halo, np.count_nonzero(halo))
    if not shares:
        return _unknown(n_lamps)

    # Warm resolves to red rather than to "one of red or yellow". Both halt the
    # cart, so the behaviour is identical either way, and calling it red is the
    # answer that stops rather than the one that shrugs.
    green_share = shares.get(LightState.GREEN, 0.0)
    warm_share = shares.get(LightState.RED, 0.0)
    state, share = (
        (LightState.GREEN, green_share) if green_share >= warm_share else (LightState.RED, warm_share)
    )
    if share < _HUE_ONLY_DOMINANCE:
        return _unknown(n_lamps)
    # Only one of the two signals spoke, so this path caps out at half
    # confidence on the same scale the two-signal path uses.
    return LightReading(
        state=state,
        confidence=_clamp01(weight * share / _MAX_SIGNAL_WEIGHT),
        lamp_index=None,
        n_lamps=n_lamps,
    )


def _inset(crop: np.ndarray) -> np.ndarray:
    height, width = crop.shape[:2]
    dy = int(height * _HOUSING_INSET)
    dx = int(width * _HOUSING_INSET)
    if height - 2 * dy < _MIN_CROP_PX or width - 2 * dx < _MIN_CROP_PX:
        return crop
    return crop[dy:height - dy, dx:width - dx]


def _lamp_cell_counts(lit: np.ndarray, n_lamps: int, vertical: bool):
    """Lit-pixel count per lamp cell, plus the slice that produced each one."""
    extent = lit.shape[0] if vertical else lit.shape[1]
    bounds = np.linspace(0, extent, n_lamps + 1).astype(int)

    counts = []
    slices = []
    for i in range(n_lamps):
        span = slice(bounds[i], bounds[i + 1])
        cell = (span, slice(None)) if vertical else (slice(None), span)
        slices.append(cell)
        counts.append(float(np.count_nonzero(lit[cell])))
    return np.array(counts), slices


def _hue_shares(hue: np.ndarray, halo: np.ndarray, emitter_px: float) -> tuple[float, dict]:
    """Vote-share per colour across the lamp's coloured ring.

    Votes rather than a median because red straddles both ends of the hue
    scale, and averaging across that wrap produces cyan.
    """
    sample = hue[halo]
    if sample.size < _MIN_HUE_PIXELS:
        return 0.0, {}

    warm = _count_in_bands(sample, _WARM_BANDS)
    green = _count_in_bands(sample, _GREEN_BANDS)
    total = warm + green
    if total == 0:
        return 0.0, {}

    # Hue is trusted in proportion to how much colour the ring carries relative
    # to the size of the emitter it surrounds. A lamp blown out so hard that
    # barely any colour survives gets little say.
    weight = _clamp01(sample.size / max(emitter_px, 1.0))

    # Red and yellow deliberately share the warm share rather than splitting
    # it. Hue is saying "not green", and leaving the rest to position.
    warm_share, green_share = warm / total, green / total
    return weight, {
        LightState.RED: warm_share,
        LightState.YELLOW: warm_share,
        LightState.GREEN: green_share,
    }


def _count_in_bands(sample: np.ndarray, bands) -> int:
    return sum(int(np.count_nonzero((sample >= low) & (sample <= high))) for low, high in bands)


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))
