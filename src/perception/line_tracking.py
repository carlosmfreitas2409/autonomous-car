from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import cv2 as cv
import numpy as np

from config import Config


_WIDTH_EMA_ALPHA = 0.2

_CALIBRATION_FRAMES = 30

_STEADY_FRAMES = 15
_STEADY_TOLERANCE = 0.03

_ROW_AGREEMENT = 0.10

_BAND_TOP = 0.35
_BAND_STEP = 12

_MIN_LINE_RUN_RATIO = 0.02
_FRAME_EDGE_RATIO = 0.03

_FAR_MIN_WIDTH = 0.2
_FAR_MIN_ROWS = 3

_ANCHOR_COLOR = (255, 128, 0)
_BAND_COLOR = (90, 90, 160)


def _median_or_none(values) -> Optional[float]:
    return float(np.median(values)) if len(values) else None


def _theil_sen(points):
    slopes = [
        (x2 - x1) / (y2 - y1)
        for i, (y1, x1) in enumerate(points)
        for (y2, x2) in points[i + 1:]
        if y2 != y1
    ]
    slope = float(np.median(slopes))
    intercept = float(np.median([x - slope * y for y, x in points]))
    return slope, intercept


def _ema_pair(prev, new, alpha):
    if prev is None:
        return new
    return (alpha * new[0] + (1 - alpha) * prev[0], alpha * new[1] + (1 - alpha) * prev[1])


class LaneAnchor(Enum):
    AUTO = "auto"
    LEFT = "left"
    RIGHT = "right"


@dataclass
class LineTrackingResult:
    display_frame: np.ndarray
    debug_frame: np.ndarray
    lane_offset: Optional[int]
    lane_curvature: Optional[float]
    lane_state: str = ""
    anchor: LaneAnchor = LaneAnchor.AUTO
    width_ratios: tuple[float, ...] = ()
    rows_found: int = 0
    both_rows: int = 0
    profile_ready: bool = False
    provisional: bool = False
    row_centers: tuple[tuple[int, int, bool], ...] = ()
    band_rows: int = 0
    far_offset: Optional[int] = None
    far_curvature: Optional[float] = None
    band_edges: tuple[tuple[int, int, int], ...] = ()
    width_profile: Optional[tuple[float, float]] = None

    @property
    def width_ratio(self) -> Optional[float]:
        return _median_or_none(self.width_ratios)


class LineTracker:
    def __init__(self):
        self._prev_track_center: Optional[int] = None
        self._smoothed_center: Optional[float] = None
        self._half_lane_width: Optional[float] = None
        self._smoothed_curvature: Optional[float] = None
        self._band_edges: tuple[tuple[int, int, int], ...] = ()

        self._calibration_profile: Optional[tuple[float, float]] = None
        self._width_profile: Optional[tuple[float, float]] = None
        self._calibration = 0

        self._anchor_side = LaneAnchor.AUTO
        self._learning = True
        self._norm_history: deque = deque(maxlen=_STEADY_FRAMES)
        self._steady_run = 0

        self.clamped_frames = 0

    def reset(self) -> None:
        self._prev_track_center = None
        self._smoothed_center = None
        self._half_lane_width = None
        self._smoothed_curvature = None
        self._norm_history.clear()
        self._steady_run = 0

    @property
    def profile_ready(self) -> bool:
        return self._calibration >= _CALIBRATION_FRAMES

    def process(self, frame: np.ndarray, params: Config, anchor: LaneAnchor = LaneAnchor.AUTO,
                learn: bool = True) -> LineTrackingResult:
        self._learning = learn
        ratio = params.downscale_ratio
        height = round(frame.shape[0] / ratio)
        width = round(frame.shape[1] / ratio)
        center_x = width // 2
        frame = cv.resize(frame, (width, height))

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        bk = max(3, params.blur_ksize | 1)
        blur = cv.medianBlur(gray, bk)

        bs = max(3, params.block_size | 1)
        binary = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, bs, int(params.threshold_constant))

        roi_x0 = round(width * max(0.0, min(params.roi_x0, params.roi_x1 - 0.02)))
        roi_x1 = round(width * min(1.0, params.roi_x1))
        roi_y0 = round(height * max(0.0, min(params.roi_y0, params.roi_y1 - 0.02)))
        roi_y1 = round(height * min(1.0, params.roi_y1))
        roi_center_x = (roi_x0 + roi_x1) // 2

        debug = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)
        cv.rectangle(debug, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 255, 0), 1)
        cv.line(debug, (roi_center_x, roi_y0), (roi_center_x, roi_y1), (255, 255, 255), 1)

        scan_ys = np.linspace(roi_y0, roi_y1, max(1, params.scan_lines), dtype=int)
        scan_origin = self._prev_track_center if self._prev_track_center is not None else center_x

        if anchor is not self._anchor_side:
            self._begin_anchor(anchor)
        scan_args = (binary, debug, scan_ys, scan_origin, roi_x0, roi_x1, height)

        provisional = False
        width_ratios: tuple[float, ...] = ()
        both_rows = 0
        far_offset: Optional[int] = None
        far_curvature: Optional[float] = None
        if self._anchor_side is LaneAnchor.AUTO or not self.profile_ready:
            centers, center_ys, widths, samples, lane_state = self._scan_auto(*scan_args)
            self._learn_width(widths)
            both_rows = len(samples)
            roi_samples = [(y, right_x - left_x) for y, left_x, right_x in samples]
            band_samples = self._scan_band(binary, debug, scan_origin, width, height, roi_y0)
            if not self.profile_ready:
                if self._learn_geometry(roi_samples, band_samples, roi_y0):
                    self._calibration += 1
                    if self.profile_ready:
                        self._calibration_profile = self._width_profile
                        self._norm_history.clear()
                        self._steady_run = 0
            else:
                self._learn_when_steady(roi_samples, band_samples)
            if self._anchor_side is not LaneAnchor.AUTO and centers:
                lane_state = f"calibrating {self._calibration}/{_CALIBRATION_FRAMES}"
            row_centers = tuple((y, c, any(s[0] == y for s in samples)) for y, c in zip(center_ys, centers))
        else:
            band_samples = self._scan_band(binary, debug, scan_origin, width, height, roi_y0)
            centers, center_ys, lane_state, width_ratios, provisional, both_rows = self._anchored_frame(scan_args, band_samples, width)
            row_centers = tuple((y, c, False) for y, c in zip(center_ys, centers))
            blind = not centers or provisional or len(centers) <= max(1, params.scan_lines) // 2
            if blind and self._anchor_side is LaneAnchor.LEFT:
                far = self._read_far(binary, debug, scan_origin, roi_x0, roi_x1, roi_y0, roi_y1, height, width)
                if far is not None:
                    far_offset, far_curvature = far
                    lane_state = f"{lane_state} far"

        lane_offset: Optional[int] = None
        lane_curvature: Optional[float] = None

        if centers and provisional:
            raw_x = int(np.median(centers))
            lane_offset = raw_x - roi_center_x
            cv.line(debug, (raw_x, roi_y0), (raw_x, roi_y1), (0, 128, 255), 1)
        elif centers:
            track_center_x = int(np.median(centers))
            self._prev_track_center = track_center_x
            if self._anchor_side is not LaneAnchor.AUTO:
                clamped = max(roi_x0, min(roi_x1 - 1, track_center_x))
                if clamped != track_center_x:
                    self.clamped_frames += 1
                self._prev_track_center = clamped

            if self._smoothed_center is None:
                self._smoothed_center = track_center_x
            else:
                self._smoothed_center = params.ema_alpha * track_center_x + (1 - params.ema_alpha) * self._smoothed_center

            smoothed_x = int(self._smoothed_center)
            lane_offset = smoothed_x - roi_center_x

            cv.line(debug, (smoothed_x, roi_y0), (smoothed_x, roi_y1), (0, 0, 255), 2)

            if len(center_ys) >= 2 and len(set(center_ys)) >= 2:
                fit = np.polyfit(center_ys, centers, 1)
                raw_curvature = -float(fit[0])

                if self._smoothed_curvature is None:
                    self._smoothed_curvature = raw_curvature
                else:
                    self._smoothed_curvature = params.ema_alpha * raw_curvature + (1 - params.ema_alpha) * self._smoothed_curvature
                lane_curvature = self._smoothed_curvature

                near_y = max(center_ys)
                far_y = min(center_ys)
                near_x = int(np.polyval(fit, near_y))
                far_x = int(np.polyval(fit, far_y))
                cv.line(debug, (near_x, near_y), (far_x, far_y), (255, 0, 255), 1)

        return LineTrackingResult(
            display_frame=frame,
            debug_frame=debug,
            lane_offset=lane_offset,
            lane_curvature=lane_curvature,
            lane_state=lane_state,
            anchor=self._anchor_side,
            width_ratios=width_ratios,
            rows_found=len(centers),
            both_rows=both_rows,
            profile_ready=self.profile_ready,
            provisional=provisional,
            row_centers=row_centers,
            band_rows=len(band_samples),
            far_offset=far_offset,
            far_curvature=far_curvature,
            band_edges=self._band_edges,
            width_profile=self._width_profile,
        )

    def _begin_anchor(self, side: LaneAnchor) -> None:
        self._anchor_side = side
        self._norm_history.clear()
        self._steady_run = 0

    def _anchored_frame(self, scan_args, band_samples, width):
        centers, center_ys, ratios, samples, rows = self._scan_anchored(*scan_args, width)
        name = f"anchor-{self._anchor_side.value}"

        if not centers:
            centers, center_ys, _widths, auto_samples, auto_state = self._scan_auto(*scan_args)
            samples = [(y, right_x - left_x) for y, left_x, right_x in auto_samples]
            ratios = [w / (2.0 * self._half_width_at(y)) for y, w in samples]
            name = f"{name} fallback {auto_state}"
        else:
            name = f"{name} {len(centers)}/{rows}"

        if not centers:
            return centers, center_ys, name, (), False, 0

        if len(centers) < 2:
            return centers, center_ys, f"{name} provisional", tuple(ratios), True, len(samples)

        self._learn_when_steady(samples, band_samples)
        return centers, center_ys, name, tuple(ratios), False, len(samples)

    def _learn_width(self, widths) -> None:
        if not self._learning:
            return
        if widths:
            measured_half = float(np.median(widths)) / 2.0
            if self._half_lane_width is None:
                self._half_lane_width = measured_half
            else:
                self._half_lane_width = _WIDTH_EMA_ALPHA * measured_half + (1 - _WIDTH_EMA_ALPHA) * self._half_lane_width

    @staticmethod
    def _geometry_samples(roi_samples, band_samples):
        if len({y for y, _ in roi_samples}) >= 2:
            return roi_samples
        return roi_samples + band_samples

    def _learn_geometry(self, roi_samples, band_samples, ref_y: int) -> bool:
        if not self._learning:
            return False
        fit = self._fit_width_profile(self._geometry_samples(roi_samples, band_samples), roi_samples)
        if fit is None:
            self._norm_history.clear()
            return False
        self._norm_history.append(self._profile_width_at(fit, ref_y))
        if not self._history_steady():
            return False
        self._width_profile = _ema_pair(self._width_profile, fit, _WIDTH_EMA_ALPHA)
        return True

    def _learn_when_steady(self, roi_samples, band_samples) -> None:
        if not self._learning:
            return
        if self._calibration_profile is None:
            return
        samples = self._geometry_samples(roi_samples, band_samples)
        if not samples:
            self._steady_run = 0
            self._norm_history.clear()
            return

        norms = [w / self._profile_width_at(self._calibration_profile, y) for y, w in samples]
        norm = float(np.median(norms))
        self._norm_history.append(norm)
        if not self._history_steady():
            self._steady_run = 0
            return
        self._steady_run += 1

        agreeing = [sample for sample, row_norm in zip(samples, norms) if abs(row_norm / norm - 1.0) <= _ROW_AGREEMENT]
        roi_agreeing = [sample for sample in roi_samples if sample in agreeing]
        fit = self._fit_width_profile(agreeing, roi_agreeing)
        if fit is not None:
            self._width_profile = _ema_pair(self._width_profile, fit, _WIDTH_EMA_ALPHA)

    def _history_steady(self) -> bool:
        return (
            len(self._norm_history) == _STEADY_FRAMES
            and min(self._norm_history) > 0
            and max(self._norm_history) / min(self._norm_history) - 1.0 <= _STEADY_TOLERANCE
        )

    @staticmethod
    def _fit_width_profile(samples, roi_samples=()):
        if len({y for y, _ in samples}) < 2:
            return None
        slope, intercept = _theil_sen(samples)
        if slope <= 0:
            return None
        if roi_samples:
            intercept = float(np.median([w - slope * y for y, w in roi_samples]))
        return slope, intercept

    @staticmethod
    def _profile_width_at(profile, y: int) -> float:
        a, b = profile
        return max(2.0, a * y + b)

    def _half_width_at(self, y: int) -> float:
        if self._width_profile is None:
            return max(1.0, self._half_lane_width or 1.0)
        return self._profile_width_at(self._width_profile, y) / 2.0

    def _scan_auto(self, binary, debug, scan_ys, scan_origin, roi_x0, roi_x1, height):
        centers = []
        center_ys = []
        widths = []
        samples = []
        rows = 0
        both = 0
        left_only = 0
        right_only = 0

        for sy in scan_ys:
            if sy < 0 or sy >= height:
                continue
            rows += 1
            left_x = -1
            right_x = -1

            for i in range(scan_origin, roi_x0, -1):
                if binary[sy, i] == 255:
                    left_x = i
                    break

            for i in range(scan_origin, roi_x1, 1):
                if binary[sy, i] == 255:
                    right_x = i
                    break

            center: Optional[int] = None
            if left_x != -1 and right_x != -1:
                center = (left_x + right_x) // 2
                widths.append(right_x - left_x)
                samples.append((int(sy), left_x, right_x))
                both += 1
                cv.line(debug, (left_x, sy), (right_x, sy), (0, 255, 255), 1)
            elif left_x != -1 and self._half_lane_width is not None:
                center = int(left_x + self._half_lane_width)
                left_only += 1
            elif right_x != -1 and self._half_lane_width is not None:
                center = int(right_x - self._half_lane_width)
                right_only += 1

            if center is not None:
                centers.append(center)
                center_ys.append(int(sy))

        if not centers:
            return centers, center_ys, widths, samples, "LOST"

        label, count = max(
            (("both", both), ("left-only", left_only), ("right-only", right_only)),
            key=lambda item: item[1],
        )
        return centers, center_ys, widths, samples, f"{label} {count}/{rows}"

    def _scan_band(self, binary, debug, scan_origin, width, height, roi_y0):
        rows = list(range(round(_BAND_TOP * height), roi_y0, _BAND_STEP))
        self._band_edges = ()
        if len(rows) < 2:
            return []
        samples = []
        edges = []
        origin = max(1, min(width - 1, int(scan_origin)))
        for y in reversed(rows):
            if y < 0 or y >= height:
                continue
            row = binary[y]
            lefts = np.flatnonzero(row[:origin] == 255)
            rights = np.flatnonzero(row[origin:] == 255)
            if not len(lefts) or not len(rights):
                if len(lefts) or len(rights):
                    edges.append((int(y), int(lefts[-1]) if len(lefts) else -1,
                                  int(rights[0]) + origin if len(rights) else -1))
                continue
            left_x = int(lefts[-1])
            right_x = int(rights[0]) + origin
            if right_x - left_x < 2 or np.all(row[left_x:right_x + 1] == 255):
                continue
            samples.append((y, right_x - left_x))
            edges.append((int(y), left_x, right_x))
            origin = max(1, min(width - 1, (left_x + right_x) // 2))
            cv.circle(debug, (left_x, y), 1, _BAND_COLOR, -1)
            cv.circle(debug, (right_x, y), 1, _BAND_COLOR, -1)
        self._band_edges = tuple(edges)
        return samples

    def _read_far(self, binary, debug, scan_origin, roi_x0, roi_x1, roi_y0, roi_y1, height, width):
        ref_width = 2.0 * self._half_width_at((roi_y0 + roi_y1) // 2)
        far_ys = [
            y for y in range(0, roi_y0, _BAND_STEP)
            if 2.0 * self._half_width_at(y) >= _FAR_MIN_WIDTH * ref_width
        ]
        if len(far_ys) < _FAR_MIN_ROWS:
            return None
        origin = max(roi_x0, min(roi_x1 - 1, int(scan_origin)))
        centers, center_ys, _ratios, _samples, _rows = self._scan_anchored(
            binary, debug, np.array(far_ys, dtype=int), origin, roi_x0, roi_x1, height, width)
        nearest = sorted(zip(center_ys, centers), reverse=True)
        if not nearest:
            return None
        window = [(y, x) for y, x in nearest if y >= nearest[0][0] - (roi_y1 - roi_y0)]
        if len(window) < _FAR_MIN_ROWS or len({y for y, _ in window}) < 2:
            return None
        ys = [y for y, _ in window]
        xs = [x for _, x in window]
        slope = float(np.polyfit(ys, xs, 1)[0])
        center_x = int(np.median(xs))
        for y, x in window:
            cv.circle(debug, (int(x), int(y)), 3, (0, 0, 255), -1)
        return center_x - (roi_x0 + roi_x1) // 2, -slope

    def _scan_anchored(self, binary, debug, scan_ys, scan_origin, roi_x0, roi_x1, height, width):
        left = self._anchor_side is LaneAnchor.LEFT
        found = []
        fallback_centers = []
        fallback_ys = []
        rows = 0
        chain_x = scan_origin
        prev_edge_x: Optional[int] = None

        for sy in scan_ys[::-1]:
            if sy < 0 or sy >= height:
                continue
            rows += 1
            row_y = int(sy)
            origin = max(roi_x0, min(roi_x1 - 1, chain_x))
            half = self._half_width_at(row_y)

            min_run = _MIN_LINE_RUN_RATIO * 2.0 * half
            if left:
                edge_x = self._first_white_left(binary, row_y, origin, -1)
                other_x = self._first_white_right(binary, row_y, origin, roi_x1)
                other_x = self._line_or_none(binary, row_y, other_x, 1, min_run)
            else:
                edge_x = self._first_white_right(binary, row_y, origin, width)
                other_x = self._first_white_left(binary, row_y, origin, roi_x0)
                other_x = self._line_or_none(binary, row_y, other_x, -1, min_run)

            if edge_x != -1 and prev_edge_x is not None:
                inward = (edge_x - prev_edge_x) if left else (prev_edge_x - edge_x)
                if inward > half:
                    edge_x = -1

            if edge_x == -1:
                if other_x != -1 and abs(other_x - origin) <= 2.0 * half:
                    center = int(other_x - half) if left else int(other_x + half)
                    fallback_centers.append(center)
                    fallback_ys.append(row_y)
                    chain_x = center
                    cv.circle(debug, (other_x, row_y), 3, (0, 255, 255), 1)
                continue

            found.append((row_y, edge_x, other_x))
            chain_x = int(edge_x + half) if left else int(edge_x - half)
            prev_edge_x = edge_x

        centers = list(fallback_centers)
        center_ys = list(fallback_ys)
        ratios = []
        samples = []
        for row_y, edge_x, other_x in found:
            half = self._half_width_at(row_y)
            center = int(edge_x + half) if left else int(edge_x - half)
            centers.append(center)
            center_ys.append(row_y)

            if other_x != -1:
                measured = abs(other_x - edge_x)
                ratios.append(measured / (2.0 * half))
                samples.append((row_y, measured))
                cv.line(debug, (edge_x, row_y), (other_x, row_y), (0, 255, 255), 1)
            cv.line(debug, (edge_x, row_y), (center, row_y), _ANCHOR_COLOR, 1)
            cv.circle(debug, (edge_x, row_y), 3, _ANCHOR_COLOR, -1)

        return centers, center_ys, ratios, samples, rows

    @staticmethod
    def _line_or_none(binary, row_y: int, x: int, step: int, min_run: float) -> int:
        if x == -1:
            return -1
        row = binary[row_y]
        width = row.shape[0]
        margin = _FRAME_EDGE_RATIO * width
        near_edge = (x >= width - margin) if step > 0 else (x <= margin)
        if not near_edge:
            return x
        n = 0
        while 0 <= x + n * step < width and row[x + n * step] == 255:
            n += 1
            if n >= min_run:
                return x
        return -1

    @staticmethod
    def _first_white_left(binary, row_y: int, origin: int, bound: int) -> int:
        for i in range(origin, bound, -1):
            if binary[row_y, i] == 255:
                return i
        return -1

    @staticmethod
    def _first_white_right(binary, row_y: int, origin: int, bound: int) -> int:
        for i in range(origin, bound, 1):
            if binary[row_y, i] == 255:
                return i
        return -1
