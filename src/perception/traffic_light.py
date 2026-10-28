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


SPACING_OVER_RADIUS = 2.15

HOUSING_MARGIN_X = 1.48
HOUSING_MARGIN_TOP = 1.53
HOUSING_MARGIN_BOTTOM = 1.15

_LEVELS = (99.5, 99, 98, 96, 93, 90, 86, 81, 75, 68, 60, 52, 44, 36, 28)

_BLOB_FILL = (0.60, 0.95)
_BLOB_ASPECT = (0.62, 1.60)

_MAX_CANDIDATES = 10

_WORK_HEIGHT = 200

_CORE = 0.72
_RING = (1.12, 1.45)
_HALO = (1.00, 2.30)

_BODY_OFFSET = 1.27
_BODY_HALF = 0.17

EMISSION_FLOOR = 0.43

CROP_MARGIN = 0.30

_SATURATED = 230
_BLOWN_SPREAD = 0.45

_RED_GLOW = 0.80

_BLOWN = 244


@dataclass
class CellStack:
    cx: float
    centres: tuple[float, float, float]
    radius: float
    contrast: tuple[float, float, float]
    clipped: tuple[int, ...] = ()
    score: float = 0.0
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
    position_evidence: float = 0.0
    colour_evidence: float = 0.0
    warm_votes: float = 0.0
    green_votes: float = 0.0
    emission: tuple[float, ...] = ()
    reason: str = ""
    debug: dict = field(default_factory=dict)

    @property
    def verified(self) -> bool:
        return self.cells is not None


def read_light_state(crop: np.ndarray) -> LightReading:
    if crop is None or crop.size == 0 or min(crop.shape[:2]) < 8:
        return LightReading(reason="crop too small")

    work, scale = _to_work_height(crop)
    reading = _read(work, scale)
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
    reading.confidence = round(0.5 * reading.confidence, 3)
    reading.reason = f"housing washed out - {reading.reason}, core at {centre:.2f}"
    reading.debug = _debug_masks(work, stack, lit)
    return reading


def _cell_offsets() -> tuple[float, float, float]:
    total = HOUSING_MARGIN_TOP + HOUSING_MARGIN_BOTTOM + 2 * SPACING_OVER_RADIUS
    return tuple((HOUSING_MARGIN_TOP + i * SPACING_OVER_RADIUS) / total for i in range(3))


def _stack_from_box(width: int, height: int) -> CellStack:
    inset = CROP_MARGIN / (1 + 2 * CROP_MARGIN)
    top, bottom = inset * height, (1 - inset) * height
    box_height = bottom - top
    total = HOUSING_MARGIN_TOP + HOUSING_MARGIN_BOTTOM + 2 * SPACING_OVER_RADIUS
    radius = box_height / total
    centres = tuple(top + offset * box_height for offset in _cell_offsets())
    return CellStack(width / 2.0, centres, radius, (0.0, 0.0, 0.0), assumed=True)


def find_cell_stack(image: np.ndarray, min_radius: float = 3.0,
                    max_radius: float = 1e9) -> Optional[CellStack]:

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
    found: list[tuple[float, float, float]] = []
    area = float(gray.size)
    for level in np.unique(np.round(np.percentile(gray, [100 - p for p in _LEVELS]))):
        mask = (gray >= level).astype(np.uint8)
        if not 0.01 <= float(mask.sum()) / area <= 0.70:
            continue
        _components(mask, found)
        _distance_peaks(mask, found)

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
    window = min(61, int(max(3, round(1.2 * peak))) | 1)
    local_max = cv.dilate(distance, np.ones((window, window), np.uint8))
    ys, xs = np.nonzero((distance >= local_max - 1e-3) & (distance > 0.35 * peak))
    for x, y in zip(xs, ys):
        found.append((float(x), float(y), float(distance[y, x])))


def _hypotheses(blobs: list[tuple[float, float, float]]):
    for upper, lower in itertools.combinations(sorted(blobs, key=lambda b: b[1]), 2):
        gap = lower[1] - upper[1]
        mean_r = 0.5 * (upper[2] + lower[2])
        if gap < 6 or abs(upper[0] - lower[0]) > 0.8 * mean_r:
            continue
        cx = 0.5 * (upper[0] + lower[0])
        if 1.2 * mean_r <= gap <= 4.0 * mean_r:
            radius = gap / SPACING_OVER_RADIUS
            yield cx, [upper[1] - gap, upper[1], lower[1]], radius
            yield cx, [upper[1], lower[1], lower[1] + gap], radius
        if 2.6 * mean_r <= gap <= 8.0 * mean_r:
            radius = gap / (2 * SPACING_OVER_RADIUS)
            yield cx, [upper[1], 0.5 * (upper[1] + lower[1]), lower[1]], radius


class _Sums:
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


def _emission_per_cell(gray: np.ndarray, stack: CellStack) -> np.ndarray:
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
    order = np.argsort(emission)[::-1]
    best, runner_up = int(order[0]), int(order[1])
    if emission[best] < EMISSION_FLOOR:
        return None, 0.0
    margin = (emission[best] - emission[runner_up]) / abs(emission[best])
    return best, float(np.clip(margin, 0.0, 1.0))


def _halo_votes(bgr: np.ndarray, stack: CellStack, lit: int) -> tuple[float, float]:
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


_BY_POSITION = (LightState.RED, LightState.YELLOW, LightState.GREEN)


def _decide(reading: LightReading, lit: int, colour_family: Optional[str]) -> None:
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

    reading.state = LightState.UNKNOWN
    reading.confidence = 0.0
    reading.reason = f"position says {by_position.value}, halo says {colour_family}"


def _to_work_height(crop: np.ndarray) -> tuple[np.ndarray, float]:
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
