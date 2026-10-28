from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2 as cv
import numpy as np

SIDES_TO_VARIANT = {5: "B", 6: "A", 7: "C"}
VARIANT_TO_SIDES = {v: k for k, v in SIDES_TO_VARIANT.items()}

HOLES_TO_VARIANT = {0: "C", 1: "A", 2: "B"}

MIN_CROP_HEIGHT = 60

CUE_CROSSOVER = 80

SIDES_ONLY_MARGIN = 1.8

_MIN_WORK_HEIGHT = 96
_MAX_WORK_HEIGHT = 224

CROP_MARGIN = 0.12

_SEEDS = ((0.50, 0.08), (0.50, 0.92), (0.08, 0.50), (0.92, 0.50),
          (0.30, 0.14), (0.70, 0.86))

_FACE_AREA = (0.30, 0.95)

_MIN_SOLIDITY = 0.90

_HARMONICS = (5, 6, 7)
_ANGLES = 256

_GLYPH = 32
_ARTWORK = Path(__file__).resolve().parent.parent.parent / "assets" / "sign_cutouts" / "delivery_sign"


@dataclass
class VariantReading:
    variant: Optional[str] = None
    confidence: float = 0.0
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


def segment_face(bgr: np.ndarray, margin: float = CROP_MARGIN) -> Optional[np.ndarray]:
    height, width = bgr.shape[:2]
    blurred = cv.GaussianBlur(bgr, (3, 3), 0)
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


def count_sides(face: np.ndarray) -> tuple[Optional[int], float, Optional[np.ndarray]]:
    contours, _ = cv.findContours(face, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if not contours:
        return None, 0.0, None
    contour = max(contours, key=cv.contourArea)
    if len(contour) < 16:
        return None, 0.0, contour

    points = _whiten(contour.reshape(-1, 2).astype(np.float32), face)
    if points is None:
        return None, 0.0, contour

    radii = np.hypot(points[:, 0], points[:, 1])
    if radii.mean() <= 1e-3:
        return None, 0.0, contour
    angles = np.mod(np.arctan2(points[:, 1], points[:, 0]), 2 * np.pi)

    order = np.argsort(angles)
    grid = np.linspace(0, 2 * np.pi, _ANGLES, endpoint=False)
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


def read_letter(bgr: np.ndarray, face: np.ndarray) -> tuple[dict, Optional[np.ndarray]]:
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
        if result["variant"] is None:
            result["variant"] = ranked[0][0]
        elif ranked[0][0] != result["variant"] and result["gap"] > 0.08:
            result["variant"] = None
    return result, normalised


def _glyph_mask(bgr: np.ndarray, face: np.ndarray) -> Optional[np.ndarray]:
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
    padded = cv.copyMakeBorder(glyph, 1, 1, 1, 1, cv.BORDER_CONSTANT, value=0)
    count, labels, stats, _ = cv.connectedComponentsWithStats(1 - padded, 4)
    outside = int(labels[0, 0])
    floor = max(2.0, 0.005 * float(glyph.sum()))
    return int(sum(1 for i in range(1, count)
                   if i != outside and stats[i, cv.CC_STAT_AREA] >= floor))


def _normalise_glyph(glyph: np.ndarray) -> np.ndarray:
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
    global _TEMPLATE_CACHE
    if _TEMPLATE_CACHE is not None:
        return _TEMPLATE_CACHE

    templates = {}
    for variant in ("a", "b", "c"):
        path = _ARTWORK / f"variant_{variant}.png"
        image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
        if image is None:
            continue
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
        if reading.sides_margin < SIDES_ONLY_MARGIN:
            reading.reason = (f"only the {reading.sides}-side count, and it won by "
                              f"{reading.sides_margin:.1f}x - not enough alone")
            return
        reading.variant = sides_variant
        reading.confidence = round(min(1.0, 0.25 * reading.sides_margin), 3)
        reading.reason = "sides only - letter unreadable"
        return
    if sides_variant != letter_variant:
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


def _to_work_height(crop: np.ndarray) -> np.ndarray:
    height = crop.shape[0]
    target = int(np.clip(height, _MIN_WORK_HEIGHT, _MAX_WORK_HEIGHT))
    if target == height:
        return crop
    scale = target / height
    interpolation = cv.INTER_CUBIC if scale > 1 else cv.INTER_AREA
    return cv.resize(crop, (max(4, int(round(crop.shape[1] * scale))), target),
                     interpolation=interpolation)
