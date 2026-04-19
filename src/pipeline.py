from dataclasses import dataclass
import cv2 as cv
import numpy as np

from pid import PIDController


@dataclass
class PipelineParams:
    kp: float = 0.01
    ki: float = 0.0
    kd: float = 0.005
    block_size: int = 51
    threshold_constant: float = -25
    blur_ksize: int = 3
    roi_y0: float = 0.8
    roi_y1: float = 0.92
    ema_alpha: float = 0.4
    scan_lines: int = 5
    autonomous: bool = True
    stopped: bool = False


@dataclass
class PipelineResult:
    display_frame: np.ndarray
    blur_frame: np.ndarray
    binary_frame: np.ndarray
    error: int
    steering: float
    fps: float


class PipelineState:
    def __init__(self, params: PipelineParams):
        self.pid = PIDController(kp=params.kp, ki=params.ki, kd=params.kd, output_limits=(-1.0, 1.0))
        self.prev_track_center = None
        self.smoothed_center = None
        self.fps = 0.0

    def reset(self):
        self.prev_track_center = None
        self.smoothed_center = None
        self.pid.reset()

    def update_pid_gains(self, params: PipelineParams):
        self.pid.kp = params.kp
        self.pid.ki = params.ki
        self.pid.kd = params.kd


def process_frame(frame: np.ndarray, params: PipelineParams, state: PipelineState, dt: float) -> PipelineResult:
    ratio = 1.5
    height = round(frame.shape[0] / ratio)
    width = round(frame.shape[1] / ratio)
    center_x = width // 2
    frame = cv.resize(frame, (width, height))

    # Update PID gains from GUI
    state.update_pid_gains(params)

    # FPS
    if dt > 0:
        state.fps = 0.9 * state.fps + 0.1 * (1.0 / dt)

    # Preprocessing
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    bk = max(3, params.blur_ksize | 1)
    blur = cv.medianBlur(gray, bk)

    bs = max(3, params.block_size | 1)
    binary = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, bs, int(params.threshold_constant))

    # ROI
    roi_x0, roi_x1 = round(width * 0.1), round(width * 0.9)
    roi_y0 = round(height * max(0.0, min(params.roi_y0, params.roi_y1 - 0.02)))
    roi_y1 = round(height * min(1.0, params.roi_y1))
    roi_center_x = (roi_x0 + roi_x1) // 2

    blur_display = cv.cvtColor(blur, cv.COLOR_GRAY2BGR)
    cv.rectangle(blur_display, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 255, 0), 1)

    # Road tracking with multiple scan lines
    scan_ys = np.linspace(roi_y0, roi_y1, max(1, params.scan_lines), dtype=int)
    centers = []
    scan_origin = state.prev_track_center if state.prev_track_center is not None else center_x

    cv.line(blur_display, (roi_center_x, roi_y0), (roi_center_x, roi_y1), (255, 255, 255), 1)

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

        if left_x != -1 and right_x != -1:
            centers.append((left_x + right_x) // 2)
            cv.line(blur_display, (left_x, sy), (right_x, sy), (0, 255, 255), 1)

    error = 0
    steering = 0.0

    if centers:
        track_center_x = int(np.median(centers))
        state.prev_track_center = track_center_x

        if state.smoothed_center is None:
            state.smoothed_center = track_center_x
        else:
            state.smoothed_center = params.ema_alpha * track_center_x + (1 - params.ema_alpha) * state.smoothed_center

        smoothed_x = int(state.smoothed_center)
        error = smoothed_x - roi_center_x

        steering = state.pid.update(error, dt)

        cv.line(blur_display, (smoothed_x, roi_y0), (smoothed_x, roi_y1), (0, 0, 255), 2)

    # Overlays on frame
    mode_label = "AUTO" if params.autonomous else "MANUAL"
    mode_color = (0, 255, 0) if params.autonomous else (0, 0, 255)
    
    cv.putText(frame, mode_label, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)

    # Steering bar
    bar_y = height - 30
    bar_w = width - 40
    bar_cx = center_x
    cv.line(frame, (20, bar_y), (20 + bar_w, bar_y), (100, 100, 100), 2)
    cv.line(frame, (bar_cx, bar_y - 8), (bar_cx, bar_y + 8), (255, 255, 255), 1)
    indicator_x = int(bar_cx + steering * (bar_w // 2))
    indicator_x = max(20, min(20 + bar_w, indicator_x))
    color = (0, 0, 255) if abs(steering) > 0.5 else (0, 255, 255) if abs(steering) > 0.2 else (0, 255, 0)
    cv.circle(frame, (indicator_x, bar_y), 6, color, -1)

    return PipelineResult(
        display_frame=frame,
        blur_frame=blur_display,
        binary_frame=cv.cvtColor(binary, cv.COLOR_GRAY2BGR),
        error=error,
        steering=steering,
        fps=state.fps,
    )
