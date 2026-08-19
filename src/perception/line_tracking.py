from dataclasses import dataclass
from typing import Optional
import cv2 as cv
import numpy as np

from config import Config


# How fast the learned lane width adapts to the current curve (EMA factor).
_WIDTH_EMA_ALPHA = 0.2


@dataclass
class LineTrackingResult:
    display_frame: np.ndarray
    blur_frame: np.ndarray
    binary_frame: np.ndarray
    lane_offset: Optional[int]
    lane_curvature: Optional[float]


class LineTracker:
    def __init__(self):
        self._prev_track_center: Optional[int] = None
        self._smoothed_center: Optional[float] = None
        self._half_lane_width: Optional[float] = None
        self._smoothed_curvature: Optional[float] = None

    def reset(self) -> None:
        self._prev_track_center = None
        self._smoothed_center = None
        self._half_lane_width = None
        self._smoothed_curvature = None

    def process(self, frame: np.ndarray, params: Config) -> LineTrackingResult:
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
        # The scan origin indexes into the row directly, so it has to stay inside
        # the ROI: a single-lane frame can extrapolate a center far off-image.
        scan_origin = self._prev_track_center if self._prev_track_center is not None else center_x
        scan_origin = int(min(max(scan_origin, roi_x0), max(roi_x0, roi_x1 - 1)))
        centers = []
        center_ys = []
        widths = []

        for sy in scan_ys:
            if sy < 0 or sy >= height:
                continue
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
                cv.line(blur_display, (left_x, sy), (right_x, sy), (0, 255, 255), 1)
            elif left_x != -1 and self._half_lane_width is not None:
                center = int(min(left_x + self._half_lane_width, roi_x1))
            elif right_x != -1 and self._half_lane_width is not None:
                center = int(max(right_x - self._half_lane_width, roi_x0))

            if center is not None:
                centers.append(center)
                center_ys.append(int(sy))

        # Learn the half-lane-width from frames where both lanes are visible so
        # the single-lane estimate stays calibrated to the current curve.
        if widths:
            measured_half = float(np.median(widths)) / 2.0
            if self._half_lane_width is None:
                self._half_lane_width = measured_half
            else:
                self._half_lane_width = _WIDTH_EMA_ALPHA * measured_half + (1 - _WIDTH_EMA_ALPHA) * self._half_lane_width

        lane_offset: Optional[int] = None
        lane_curvature: Optional[float] = None

        if centers:
            track_center_x = int(np.median(centers))
            self._prev_track_center = track_center_x

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
        )
