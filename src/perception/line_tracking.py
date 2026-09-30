from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import cv2 as cv
import numpy as np

from config import Config


# How fast the learned lane width adapts to the current curve (EMA factor).
# Shared by the scalar half-width the two-edge scan falls back on and by the
# per-row width profile the anchored scan places its centers with.
_WIDTH_EMA_ALPHA = 0.2

# Frames that taught the width profile before an anchored scan may use it.
_CALIBRATION_FRAMES = 30

# The lane has a width once the measured width has sat within this tolerance
# for this many frames in a row. That is also when the width is learned:
# nothing is assumed about how wide a lane is, only that a lane that is
# steadily some width *is* that width, while a width that is still changing
# (a fork opening, a merge) is nobody's lane yet and teaches nothing.
_STEADY_FRAMES = 15
_STEADY_TOLERANCE = 0.03

# Only rows that agree with the frame's median width to this tolerance teach
# it. The far row reaches across a gap a good second before the others do;
# one such row in five is harmless to the median but not to a line fit.
_ROW_AGREEMENT = 0.10

# Band rows: the frame between this fraction of its height and the top of the
# ROI, sampled every `_BAND_STEP` rows. On this camera (wide angle, looking
# steeply down) the lane spans ~90 % of the frame at the far ROI row and the
# near rows almost never see a line, so the ROI alone has one both-lines row
# per frame and cannot fit a width-vs-row line. The band above it sees both
# lines in every main-road frame. It only ever measures the lane; it never
# steers.
_BAND_TOP = 0.35
_BAND_STEP = 12

# Colour of the single edge followed in an anchored mode (BGR).
_ANCHOR_COLOR = (255, 128, 0)
_BAND_COLOR = (90, 90, 160)


def _median_or_none(values) -> Optional[float]:
    return float(np.median(values)) if len(values) else None


def _theil_sen(points):
    """(slope, intercept) of x against y, robust to one bad point in five.

    The far scan row regularly catches an edge that is not the lane, and one
    such row is enough to flatten a least-squares slope across five rows.
    `points` are (y, x) pairs at two or more distinct y.
    """
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
    """Which lane edge the tracker follows.

    AUTO is the two-edge scan (the midpoint between the first white pixel on
    each side). LEFT and RIGHT place every row's center one half-lane inside
    that edge, using a lane width learned per image row. See `path.py` for
    which one the operator's path choice selects.
    """

    AUTO = "auto"
    LEFT = "left"
    RIGHT = "right"


@dataclass
class LineTrackingResult:
    display_frame: np.ndarray
    blur_frame: np.ndarray
    binary_frame: np.ndarray
    lane_offset: Optional[int]
    lane_curvature: Optional[float]
    lane_state: str = ""
    anchor: LaneAnchor = LaneAnchor.AUTO
    # Anchored modes only: per row that saw the other edge, measured width over
    # the width the lane has steadily had. About 1.0 on a whole lane. Empty in AUTO.
    width_ratios: tuple[float, ...] = ()
    # Telemetry and controller gating only; nothing below feeds the offset.
    rows_found: int = 0
    both_rows: int = 0
    profile_ready: bool = False
    # An anchored frame that found fewer than two rows: the offset is
    # reported, but the tracker's memory did not move and the curvature is
    # None. The DETOUR control profile ignores such frames while it is holding.
    provisional: bool = False
    # (row y, center x, both edges seen) for every resolved scan row.
    row_centers: tuple[tuple[int, int, bool], ...] = ()
    # Band rows above the ROI that measured the lane this frame.
    band_rows: int = 0

    @property
    def width_ratio(self) -> Optional[float]:
        """Median of `width_ratios`, or None when no row saw both edges."""
        return _median_or_none(self.width_ratios)


class LineTracker:
    def __init__(self):
        self._prev_track_center: Optional[int] = None
        self._smoothed_center: Optional[float] = None
        self._half_lane_width: Optional[float] = None
        self._smoothed_curvature: Optional[float] = None

        # Lane width as a function of the image row, `width = a*y + b`. On a
        # flat road with a fixed camera perspective makes that exactly linear.
        # One number for every row tilts the single-edge centerline, and the
        # curvature fit turns the tilt into a fake bend worth 40 px of steering
        # error, nearly full lock, applied constantly.
        #
        # `_calibration_profile` is the shape learned at bootstrap and then
        # frozen; it only normalises widths so "steady" can be judged.
        # `_width_profile` is the working width, relearned whenever the lane is
        # steadily some width: the main lane, a detour, whatever is there.
        #
        # Both are camera geometry, not lane state: `reset()` keeps them.
        self._calibration_profile: Optional[tuple[float, float]] = None
        self._width_profile: Optional[tuple[float, float]] = None
        self._calibration = 0

        self._anchor_side = LaneAnchor.AUTO
        self._norm_history: deque = deque(maxlen=_STEADY_FRAMES)
        self._steady_run = 0

        # Telemetry: anchored frames whose center had to be clamped back into
        # the ROI so the next scan starts inside the frame.
        self.clamped_frames = 0

    def reset(self) -> None:
        """Forget the lane, keep the camera geometry.

        A frame-read failure or a `--video` rewind must not turn a Detour into
        an 8 s LOST hold while the profile recalibrates. Recalibration is the
        operator's call: restart the tuner on a straight.
        """
        self._prev_track_center = None
        self._smoothed_center = None
        self._half_lane_width = None
        self._smoothed_curvature = None
        self._norm_history.clear()
        self._steady_run = 0

    @property
    def profile_ready(self) -> bool:
        return self._calibration >= _CALIBRATION_FRAMES

    def process(self, frame: np.ndarray, params: Config, anchor: LaneAnchor = LaneAnchor.AUTO) -> LineTrackingResult:
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

        blur_display = cv.cvtColor(blur, cv.COLOR_GRAY2BGR)
        cv.rectangle(blur_display, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 255, 0), 1)
        cv.line(blur_display, (roi_center_x, roi_y0), (roi_center_x, roi_y1), (255, 255, 255), 1)

        scan_ys = np.linspace(roi_y0, roi_y1, max(1, params.scan_lines), dtype=int)
        scan_origin = self._prev_track_center if self._prev_track_center is not None else center_x

        if anchor is not self._anchor_side:
            self._begin_anchor(anchor)
        scan_args = (binary, blur_display, scan_ys, scan_origin, roi_x0, roi_x1, height)

        provisional = False
        width_ratios: tuple[float, ...] = ()
        both_rows = 0
        if self._anchor_side is LaneAnchor.AUTO or not self.profile_ready:
            # Anchoring on one edge needs the lane geometry to place the center
            # from, so the two-edge scan runs until enough frames have taught
            # it. It also runs whenever AUTO is selected. Rows are resolved
            # with the previous frame's scalar width first, then the width is
            # learned: that ordering is the working path, byte for byte.
            centers, center_ys, widths, samples, lane_state = self._scan_auto(*scan_args)
            self._learn_width(widths)
            both_rows = len(samples)
            roi_samples = [(y, right_x - left_x) for y, left_x, right_x in samples]
            band_samples = self._scan_band(binary, blur_display, scan_origin, width, height, roi_y0)
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
            band_samples = self._scan_band(binary, blur_display, scan_origin, width, height, roi_y0)
            centers, center_ys, lane_state, width_ratios, provisional, both_rows = self._anchored_frame(scan_args, band_samples, width)
            row_centers = tuple((y, c, False) for y, c in zip(center_ys, centers))

        lane_offset: Optional[int] = None
        lane_curvature: Optional[float] = None

        if centers and provisional:
            # One row is a bearing, not a lane: report where it points but do
            # not let it move the smoothing, the scan origin or the curvature.
            raw_x = int(np.median(centers))
            lane_offset = raw_x - roi_center_x
            cv.line(blur_display, (raw_x, roi_y0), (raw_x, roi_y1), (0, 128, 255), 1)
        elif centers:
            track_center_x = int(np.median(centers))
            self._prev_track_center = track_center_x
            if self._anchor_side is not LaneAnchor.AUTO:
                # An anchored center is one edge plus half a lane and can land
                # outside the frame; the next frame's scan must start inside it.
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

            cv.line(blur_display, (smoothed_x, roi_y0), (smoothed_x, roi_y1), (0, 0, 255), 2)

            # Curvature = lateral trend of the centerline across the ROI height,
            # expressed as the slope dx/dy (x-pixels per y-pixel). As a ratio it
            # is resolution-independent. A positive value means the lane bends
            # right ahead, anticipating the bend before the car reaches it.
            if len(center_ys) >= 2 and len(set(center_ys)) >= 2:
                fit = np.polyfit(center_ys, centers, 1)
                # scan_ys increase downward (near the car is the larger y), so
                # negate to make "x shift looking further ahead" positive-for-right.
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
                cv.line(blur_display, (near_x, near_y), (far_x, far_y), (255, 0, 255), 1)

        return LineTrackingResult(
            display_frame=frame,
            blur_frame=blur_display,
            binary_frame=cv.cvtColor(binary, cv.COLOR_GRAY2BGR),
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
        )

    # --- anchor state ----------------------------------------------------

    def _begin_anchor(self, side: LaneAnchor) -> None:
        # The working width carries over: it is the lane the car is in. It is
        # deliberately not swapped when the anchor changes: through the U-turn
        # after a merge only the outer line is in view, and the width the car
        # last steadily measured is the only width there is.
        self._anchor_side = side
        self._norm_history.clear()
        self._steady_run = 0

    def _anchored_frame(self, scan_args, band_samples, width):
        """One anchored frame: single-edge centers, with the two-edge midpoint
        as the fail-safe for when neither edge is anywhere in this frame.

        Returns centers, their rows, the state label, the width ratios, the
        provisional flag and the number of rows that measured the lane.
        """
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
            # Coming out of a blind stretch the first thing the ROI meets is a
            # line clipping one corner of it: on detour_exit.mov twelve one-row
            # frames read -7 to -73 px (full left) while the car still had to
            # hold full lock. Such a frame yields an offset (this camera's
            # normal main road is two rows, so a one-row frame must not stop
            # the car in Straight) but teaches nothing and moves nothing.
            return centers, center_ys, f"{name} provisional", tuple(ratios), True, len(samples)

        self._learn_when_steady(samples, band_samples)
        return centers, center_ys, name, tuple(ratios), False, len(samples)

    # --- lane geometry ---------------------------------------------------

    def _learn_width(self, widths) -> None:
        # Learn the half-lane-width from frames where both lanes are visible so
        # the single-lane estimate stays calibrated to the current curve.
        if widths:
            measured_half = float(np.median(widths)) / 2.0
            if self._half_lane_width is None:
                self._half_lane_width = measured_half
            else:
                self._half_lane_width = _WIDTH_EMA_ALPHA * measured_half + (1 - _WIDTH_EMA_ALPHA) * self._half_lane_width

    @staticmethod
    def _geometry_samples(roi_samples, band_samples):
        """The rows that teach the width profile this frame.

        The ROI rows are where centres are placed, so when they can fit a
        line by themselves (two distinct heights) they are the reference.
        The band only extends the fit when the ROI cannot: on this camera
        the main road has one both-lines ROI row per frame, while the old
        track's camera sees the horizon inside the band and its rows there
        are not the lane.
        """
        if len({y for y, _ in roi_samples}) >= 2:
            return roi_samples
        return roi_samples + band_samples

    def _learn_geometry(self, roi_samples, band_samples, ref_y: int) -> bool:
        """Bootstrap: one frame toward the width profile. Returns whether it counted.

        The frame's own fit must have a positive slope (perspective widens
        the lane toward the car) and the width it predicts at the far ROI row
        must have been steady for `_STEADY_FRAMES`: a camera still being
        placed, or a fork sliding past, must not calibrate.
        """
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
        """Post-bootstrap learning, AUTO or anchored, ROI and band alike.

        Samples are (y, width) rows that measured the lane this frame. The
        width is relearned only once it has been steady, from the rows that
        agree with the frame's median; a frame with nothing to measure ends
        the steady run.
        """
        if self._calibration_profile is None:
            return
        samples = self._geometry_samples(roi_samples, band_samples)
        if not samples:
            self._steady_run = 0
            self._norm_history.clear()
            return

        # Steadiness is judged against the calibration shape, which never
        # moves, so learning the width does not itself look like a change.
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
        """This frame's `width = a*y + b`, or None when the rows cannot fit one.

        Perspective always widens the lane toward the car, so a fit that does
        not is junk (two rows on the wrong things) and teaches nothing. When
        ROI rows are among the samples the intercept is re-anchored on them:
        a band fit over-predicts the near rows by 2-5 % under this lens's
        barrel distortion, and the near rows are where centres are placed.
        """
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
        """Half the lane width the working profile expects at image row `y`.

        Falls back to the scalar width when no profile exists, which is the
        only case that still carries the tilt.
        """
        if self._width_profile is None:
            # A single scan row sitting on a white pixel measures a zero width.
            return max(1.0, self._half_lane_width or 1.0)
        return self._profile_width_at(self._width_profile, y) / 2.0

    # --- scans -----------------------------------------------------------

    def _scan_auto(self, binary, blur_display, scan_ys, scan_origin, roi_x0, roi_x1, height):
        """Scan every row outward from the same origin and split the two lines.

        Returns the centers, their rows, the both-lines widths (what the scalar
        EMA learns from), the both-lines rows as (y, left_x, right_x) for the
        geometry, and the lane_state label.
        """
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
                cv.line(blur_display, (left_x, sy), (right_x, sy), (0, 255, 255), 1)
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

    def _scan_band(self, binary, blur_display, scan_origin, width, height, roi_y0):
        """Measure the lane in the rows above the ROI. Returns (y, width) samples.

        Rows are chained upward from the scan origin: each row's midpoint is
        the next row's origin, so the band follows the lane into a bend.
        Rows whose two hits belong to the same white blob (the origin sat on
        a line) are skipped. The band never produces a center; it only tells
        the geometry how wide the lane is where the ROI cannot see both lines.
        """
        rows = list(range(round(_BAND_TOP * height), roi_y0, _BAND_STEP))
        if len(rows) < 2:
            return []
        samples = []
        origin = max(1, min(width - 1, int(scan_origin)))
        for y in reversed(rows):
            if y < 0 or y >= height:
                continue
            row = binary[y]
            lefts = np.flatnonzero(row[:origin] == 255)
            rights = np.flatnonzero(row[origin:] == 255)
            if not len(lefts) or not len(rights):
                continue
            left_x = int(lefts[-1])
            right_x = int(rights[0]) + origin
            if right_x - left_x < 2 or np.all(row[left_x:right_x + 1] == 255):
                continue
            samples.append((y, right_x - left_x))
            origin = max(1, min(width - 1, (left_x + right_x) // 2))
            cv.circle(blur_display, (left_x, y), 1, _BAND_COLOR, -1)
            cv.circle(blur_display, (right_x, y), 1, _BAND_COLOR, -1)
        return samples

    def _scan_anchored(self, binary, blur_display, scan_ys, scan_origin, roi_x0, roi_x1, height, width):
        """Follow a single lane edge, placing the center one half-lane inside it.

        Rows are walked near-to-far and each one starts its scan from the
        previous row's center, so once the chain is inside a detour it keeps
        finding that detour's edge instead of the line that reappears across
        the wedge. Two guards on the chain:

        - an edge that jumps *inward* by more than half a lane from the row
          before is not this line (a reflection inside the lane, or the other
          line when the lane sweeps across the image); the row is dropped;
        - a row where the anchored edge is missing but the other edge sits
          within a lane of the chain uses that edge instead, so a bend that
          takes the inner line out of the frame does not empty the far rows.

        The anchored edge is searched all the way to the frame edge: at a
        fork the peeling line sits in the last 13-22 px the ROI slider cuts
        off. The other edge keeps the ROI bounds; it is only scanned to
        measure the lane and never moves an anchored row's center. Returns
        centers, their rows, the width ratios, the (y, width) samples behind
        them, and the row count.
        """
        left = self._anchor_side is LaneAnchor.LEFT
        found = []          # (row_y, edge_x, other_x)
        fallback_centers = []
        fallback_ys = []
        rows = 0
        chain_x = scan_origin
        prev_edge_x: Optional[int] = None

        # scan_ys increase downward, so the last row is the one nearest the car.
        for sy in scan_ys[::-1]:
            if sy < 0 or sy >= height:
                continue
            rows += 1
            row_y = int(sy)
            # A single-edge center can land outside the ROI, so the next row's
            # scan has to start from a column that is inside it.
            origin = max(roi_x0, min(roi_x1 - 1, chain_x))
            half = self._half_width_at(row_y)

            if left:
                edge_x = self._first_white_left(binary, row_y, origin, -1)
                other_x = self._first_white_right(binary, row_y, origin, roi_x1)
            else:
                edge_x = self._first_white_right(binary, row_y, origin, width)
                other_x = self._first_white_left(binary, row_y, origin, roi_x0)

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
                    cv.circle(blur_display, (other_x, row_y), 3, (0, 255, 255), 1)
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
                cv.line(blur_display, (edge_x, row_y), (other_x, row_y), (0, 255, 255), 1)
            cv.line(blur_display, (edge_x, row_y), (center, row_y), _ANCHOR_COLOR, 1)
            cv.circle(blur_display, (edge_x, row_y), 3, _ANCHOR_COLOR, -1)

        return centers, center_ys, ratios, samples, rows

    @staticmethod
    def _first_white_left(binary, row_y: int, origin: int, bound: int) -> int:
        """First white pixel at or left of `origin`, stopping before `bound`."""
        for i in range(origin, bound, -1):
            if binary[row_y, i] == 255:
                return i
        return -1

    @staticmethod
    def _first_white_right(binary, row_y: int, origin: int, bound: int) -> int:
        """First white pixel at or right of `origin`, stopping before `bound`."""
        for i in range(origin, bound, 1):
            if binary[row_y, i] == 255:
                return i
        return -1
