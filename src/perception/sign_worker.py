"""Sign perception on its own thread, with its own camera.

The control loop must never wait on inference, a camera read or a drawing
call. So all of that happens here, on a daemon thread, and the only thing the
loop ever does is take a lock and read the latest `SignSnapshot`. A snapshot is
never mutated after it is published; the tracker hands out fresh `Track`
values, so the loop can hold one for as long as it likes.

A missing model or camera is an ordinary state: the snapshot says so, the pane
shows a red banner, and the policy sees no tracks. The car drives as before.

`SIGN_CAMERA`, if set, replaces the camera index list. It takes an index or a
video path, so the whole live stack runs on a clip at a desk. A bench aid, not
a tuning knob.
"""

from dataclasses import dataclass, field
from typing import Optional
import logging
import os
import threading
import time

import cv2 as cv
import numpy as np

from perception.sign_detection import SignDetector, letterbox
from perception.sign_tracker import SignTracker, Track
from perception.traffic_light import LightState

logger = logging.getLogger(__name__)


# Fixed pane size. The GUI never cares what the camera negotiated.
DEBUG_WIDTH, DEBUG_HEIGHT = 640, 360

_CAMERA_RETRY_S = 2.0

# Copied from tools/inspect_signs.py: src/ never imports from tools/.
STATE_COLOUR = {
    LightState.RED: (60, 60, 235),
    LightState.YELLOW: (40, 210, 235),
    LightState.GREEN: (80, 210, 90),
    LightState.UNKNOWN: (150, 150, 150),
}
CLASS_COLOUR = {
    "delivery_sign": (235, 190, 90),
    "detour": (60, 200, 245),
    "stop": (70, 70, 240),
    "traffic_light": (200, 200, 200),
}
GREY = (110, 110, 110)
BANNER_RED = (0, 0, 160)


@dataclass
class SignSnapshot:
    tracks: list[Track] = field(default_factory=list)
    frame: Optional[np.ndarray] = None
    timestamp: float = 0.0          # time.monotonic() of the frame; 0 = never
    camera_open: bool = False
    detector_available: bool = False
    status: str = ""
    fps: float = 0.0

    def age(self, now: float) -> float:
        if self.timestamp <= 0.0:
            return float("inf")
        return now - self.timestamp


def render_overlay(frame: Optional[np.ndarray], tracks: list[Track], *,
                   detector_available: bool, camera_open: bool,
                   status: str = "", fps: float = 0.0) -> np.ndarray:
    """The 640x360 pane: the camera frame with boxes, or a banner.

    Rejected and unconfirmed tracks are drawn thin and grey with their reason,
    so a suppressed detection is visible as a rejection rather than absent.
    """
    if frame is None or frame.size == 0 or not camera_open:
        canvas = np.full((DEBUG_HEIGHT, DEBUG_WIDTH, 3), 24, np.uint8)
        scale, pad_x, pad_y = 1.0, 0, 0
    else:
        canvas, scale, pad_x, pad_y = letterbox(frame, DEBUG_WIDTH, DEBUG_HEIGHT, pad_value=24)

    for track in tracks:
        x0, y0, x1, y1 = track.bbox
        p0 = (int(x0 * scale + pad_x), int(y0 * scale + pad_y))
        p1 = (int(x1 * scale + pad_x), int(y1 * scale + pad_y))
        colour, thickness = _track_style(track)
        cv.rectangle(canvas, p0, p1, colour, thickness)
        cv.putText(canvas, _caption(track), (p0[0], max(12, p0[1] - 5)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1)

    if not (camera_open and detector_available):
        cv.rectangle(canvas, (0, DEBUG_HEIGHT // 2 - 26), (DEBUG_WIDTH, DEBUG_HEIGHT // 2 + 26),
                     BANNER_RED, -1)
        cv.putText(canvas, "SIGN DETECTION UNAVAILABLE", (48, DEBUG_HEIGHT // 2 - 2),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv.putText(canvas, status[:80], (48, DEBUG_HEIGHT // 2 + 18),
                   cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    elif fps > 0:
        cv.putText(canvas, f"{fps:.1f} fps", (DEBUG_WIDTH - 70, 16),
                   cv.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1)
    return canvas


def _track_style(track: Track):
    if track.rejected or not track.confirmed:
        return GREY, 1
    if track.label == "traffic_light":
        return STATE_COLOUR[track.light_state], 2
    return CLASS_COLOUR.get(track.label, (200, 200, 200)), 2


def _caption(track: Track) -> str:
    text = f"#{track.id} {track.label} {track.confidence:.2f}"
    if track.rejected:
        return f"{text} {track.light_reason}"
    if not track.confirmed:
        return f"{text} unconfirmed"
    if track.label == "traffic_light":
        text += f" -> {track.light_state.value}"
    elif track.label == "delivery_sign":
        text += f" -> {track.variant}" if track.variant else " -> ?"
    if track.distance_m is not None:
        text += f" d~{track.distance_m:.1f}m"
    return text


def _parse_source(indices) -> list:
    override = os.environ.get("SIGN_CAMERA")
    if override:
        return [int(override) if override.isdigit() else override]
    return list(indices)


class SignWorker:
    def __init__(self, camera_indices=(1, 2, 3), model_path: str = "models/signs.onnx",
                 confidence: float = 0.5):
        self._sources = _parse_source(camera_indices)
        self._model_path = model_path
        self._confidence = confidence
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sign-perception", daemon=True)
        self._snapshot = SignSnapshot(
            frame=render_overlay(None, [], detector_available=False, camera_open=False,
                                 status="starting..."),
            status="starting...",
        )

    def start(self) -> None:
        self._thread.start()

    def get_snapshot(self) -> SignSnapshot:
        with self._lock:
            return self._snapshot

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)

    # ------------------------------------------------------------------

    def _publish(self, **fields) -> None:
        previous = self.get_snapshot()
        merged = {
            "tracks": previous.tracks, "frame": previous.frame, "timestamp": previous.timestamp,
            "camera_open": previous.camera_open, "detector_available": previous.detector_available,
            "status": previous.status, "fps": previous.fps,
        }
        merged.update(fields)
        snapshot = SignSnapshot(**merged)
        with self._lock:
            self._snapshot = snapshot

    def _run(self) -> None:
        detector = SignDetector(self._model_path, confidence=self._confidence)
        detector_status = "" if detector.available else f"no model at {self._model_path}"
        tracker = SignTracker()

        capture = None
        source = None
        frame_period = 0.0
        fps = 0.0
        last_frame_time = None
        next_retry = 0.0

        try:
            while not self._stop.is_set():
                if capture is None:
                    now = time.monotonic()
                    if now < next_retry:
                        time.sleep(0.05)
                        continue
                    capture, source = self._open_camera()
                    if capture is None:
                        tried = ",".join(str(s) for s in self._sources)
                        self._publish(
                            tracks=[], camera_open=False, detector_available=detector.available,
                            status=f"NO CAMERA (tried {tried})", fps=0.0,
                            frame=render_overlay(None, [], detector_available=detector.available,
                                                 camera_open=False,
                                                 status=f"no camera (tried {tried})"),
                        )
                        tracker.reset()
                        next_retry = now + _CAMERA_RETRY_S
                        continue
                    if isinstance(source, str):
                        source_fps = capture.get(cv.CAP_PROP_FPS) or 0.0
                        frame_period = 1.0 / source_fps if source_fps > 0 else 0.0
                    last_frame_time = None

                ok, frame = capture.read()
                if not ok or frame is None:
                    if isinstance(source, str):
                        capture.set(cv.CAP_PROP_POS_FRAMES, 0)
                        ok, frame = capture.read()
                    if not ok or frame is None:
                        logger.warning("Sign camera %s stopped delivering frames - reopening", source)
                        capture.release()
                        capture = None
                        tracker.reset()
                        self._publish(tracks=[], camera_open=False, fps=0.0,
                                      status="camera read failed - reopening",
                                      frame=render_overlay(None, [], detector_available=detector.available,
                                                           camera_open=False,
                                                           status="camera read failed - reopening"))
                        next_retry = time.monotonic() + _CAMERA_RETRY_S
                        continue

                now = time.monotonic()
                if last_frame_time is not None:
                    dt = now - last_frame_time
                    if dt > 0:
                        fps = 0.8 * fps + 0.2 / dt
                last_frame_time = now

                detections = detector.detect(frame) if detector.available else []
                tracks = tracker.update(detections, frame, now)
                canvas = render_overlay(frame, tracks, detector_available=detector.available,
                                        camera_open=True, status=detector_status, fps=fps)
                self._publish(tracks=tracks, frame=canvas, timestamp=now, camera_open=True,
                              detector_available=detector.available, status=detector_status, fps=fps)

                if frame_period > 0:
                    spent = time.monotonic() - now
                    if spent < frame_period:
                        time.sleep(frame_period - spent)
        finally:
            if capture is not None:
                capture.release()

    def _open_camera(self):
        for source in self._sources:
            capture = cv.VideoCapture(source)
            if not capture.isOpened():
                capture.release()
                continue
            if not isinstance(source, str):
                _configure_camera(capture, source)
            ok, frame = capture.read()
            if not ok or frame is None:
                logger.warning("Sign camera %s opened but gave no frame", source)
                capture.release()
                continue
            height, width = frame.shape[:2]
            logger.info("Sign camera %s open at %dx%d", source, width, height)
            if abs(width / max(1, height) - 16 / 9) > 0.02:
                logger.warning("Sign camera aspect is %dx%d, not 16:9 - the %.1f deg HFOV "
                               "and every distance are off", width, height, 63.3)
            return capture, source
        return None, None


def _configure_camera(capture: cv.VideoCapture, index) -> None:
    # AVFoundation ignores some of these silently, so each result is logged.
    settings = (
        ("fourcc MJPG", cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG")),
        ("width 1280", cv.CAP_PROP_FRAME_WIDTH, 1280),
        ("height 720", cv.CAP_PROP_FRAME_HEIGHT, 720),
        # Otherwise the driver queues frames and detections lag the road.
        ("buffersize 1", cv.CAP_PROP_BUFFERSIZE, 1),
        ("autofocus off", cv.CAP_PROP_AUTOFOCUS, 0),
        # The light reader was measured on this camera's raw colour.
        ("auto white balance off", cv.CAP_PROP_AUTO_WB, 0),
    )
    for name, prop, value in settings:
        accepted = capture.set(prop, value)
        logger.info("Sign camera %s: %s -> %s", index, name, "ok" if accepted else "ignored")
