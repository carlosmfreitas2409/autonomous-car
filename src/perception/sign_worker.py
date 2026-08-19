"""Sign perception on its own camera and its own thread.

The Mega's watchdog is 500 ms and the only thing feeding it is the motor
command the control loop emits once per iteration. Anything that can push a
single iteration past that budget drops the car into WATCHDOG_IDLE, so no part
of sign perception is allowed to run inside that loop.

This worker therefore owns everything end to end — the sign camera, capture,
inference, light reading, tracking, and the debug overlay — and hands the
control loop nothing but a finished snapshot behind a lock. The control loop's
entire cost is one lock acquisition and a reference copy.

The same isolation covers failure. The camera opens inside the thread, so a
camera that takes seconds to enumerate (or never appears at all) delays
nothing; a camera unplugged mid-drive is retried in the background. In every
one of those cases the snapshot simply goes stale and the car keeps driving.
"""

from dataclasses import dataclass, field
from typing import Optional
import logging
import threading
import time
import cv2 as cv
import numpy as np

from perception.sign_detection import Detection, SignDetector, letterbox
from perception.tracking import DetectionTracker, SIGN_CAM_HFOV_DEG, Track
from perception.traffic_light import LightState, read_light_state

logger = logging.getLogger(__name__)


# Capture resolution. The OV5693 can deliver 5 MP, but decoding that at frame
# rate buys nothing: the model sees 640 px anyway, and a smaller frame keeps
# USB bandwidth and JPEG decode down.
_CAPTURE_WIDTH = 1280
_CAPTURE_HEIGHT = 720

# Autofocus hunting blurs frames at exactly the wrong moment, and a focus that
# moves also moves the focal length the distance estimate depends on. Fixed
# focus near infinity is what we want; the value is driver-dependent and
# ignored by cameras that do not support the control.
_FIXED_FOCUS = 0

# Debug pane size. Fixed so the GUI never has to care what resolution the sign
# camera actually negotiated.
DEBUG_WIDTH = 640
DEBUG_HEIGHT = 360

# How long to wait before retrying a camera that failed to open or read.
_REOPEN_INTERVAL_S = 2.0

# Idle sleep when detection is switched off, so the thread stays responsive
# to being re-enabled without spinning a core.
_IDLE_SLEEP_S = 0.05

_BOX_COLOURS = {
    "stop": (60, 60, 220),
    "detour": (220, 160, 60),
    "traffic_light": (200, 200, 200),
    "pickup_a": (120, 220, 120),
    "pickup_b": (220, 120, 220),
    "pickup_c": (120, 220, 220),
}

_LIGHT_COLOURS = {
    LightState.RED: (60, 60, 220),
    LightState.YELLOW: (60, 210, 230),
    LightState.GREEN: (80, 200, 80),
    LightState.UNKNOWN: (160, 160, 160),
}


@dataclass
class SignSnapshot:
    """What the control loop sees. Immutable by convention once published."""

    tracks: list[Track] = field(default_factory=list)
    debug_frame: Optional[np.ndarray] = None
    timestamp: float = 0.0
    camera_open: bool = False
    detector_available: bool = False
    fps: float = 0.0

    def age(self, now: float) -> float:
        return float("inf") if self.timestamp == 0.0 else now - self.timestamp


class SignWorker:
    def __init__(
        self,
        camera_index: int,
        model_path: str,
        hfov_deg: float = SIGN_CAM_HFOV_DEG,
        confidence: float = 0.45,
        enabled: bool = True,
    ):
        self._camera_index = camera_index
        self._model_path = model_path
        self._hfov_deg = hfov_deg

        # Plain attribute assignment of a single scalar is atomic in CPython,
        # so these need no lock. Note what is *not* here: the previous version
        # passed the live Config object into the thread, which meant inference
        # could read half-updated GUI values mid-frame.
        self.confidence = confidence
        self.enabled = enabled

        self._lock = threading.Lock()
        self._snapshot = SignSnapshot()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="sign-perception", daemon=True)
        self._thread.start()

    def get_snapshot(self) -> SignSnapshot:
        with self._lock:
            return self._snapshot

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # A blocking read can outlast this; the thread is a daemon so a
            # slow camera cannot keep the process alive.
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        detector = SignDetector(self._model_path, self.confidence)
        tracker = DetectionTracker(self._hfov_deg)

        capture: Optional[cv.VideoCapture] = None
        next_open_attempt = 0.0
        fps = 0.0
        previous = time.monotonic()

        while not self._stop.is_set():
            now = time.monotonic()

            if capture is None:
                if now < next_open_attempt:
                    self._stop.wait(_IDLE_SLEEP_S)
                    continue
                capture = _open_camera(self._camera_index)
                next_open_attempt = now + _REOPEN_INTERVAL_S
                if capture is None:
                    self._publish(SignSnapshot(timestamp=now, detector_available=detector.available))
                    continue

            ok, frame = capture.read()
            if not ok or frame is None:
                logger.warning("Sign camera %s stopped delivering frames", self._camera_index)
                capture.release()
                capture = None
                tracker.reset()
                self._publish(SignSnapshot(timestamp=now, detector_available=detector.available))
                continue

            elapsed = now - previous
            previous = now
            if elapsed > 0:
                fps = 0.9 * fps + 0.1 / elapsed

            if self.enabled and detector.available:
                detector.set_confidence(self.confidence)
                detections = detector.detect(frame)
                _read_light_states(frame, detections)
                tracks = tracker.update(detections, frame.shape[1], now)
            else:
                tracker.reset()
                tracks = []

            self._publish(
                SignSnapshot(
                    tracks=tracks,
                    debug_frame=render_debug_frame(frame, tracks),
                    timestamp=now,
                    camera_open=True,
                    detector_available=detector.available,
                    fps=fps,
                )
            )

            if not self.enabled:
                self._stop.wait(_IDLE_SLEEP_S)

        if capture is not None:
            capture.release()

    def _publish(self, snapshot: SignSnapshot) -> None:
        with self._lock:
            self._snapshot = snapshot


def _read_light_states(frame: np.ndarray, detections: list[Detection]) -> None:
    """Fill in each traffic light's state from its own crop.

    Deliberately not the model's job. Colour is a question we can answer
    exactly, test against synthetic images, and fix without retraining.
    """
    height, width = frame.shape[:2]
    for detection in detections:
        if detection.label != "traffic_light":
            continue
        x0, y0, x1, y1 = detection.bbox
        crop = frame[max(0, y0):min(height, y1), max(0, x0):min(width, x1)]
        reading = read_light_state(crop)
        detection.light_state = reading.state
        detection.light_confidence = reading.confidence


def render_debug_frame(frame: np.ndarray, tracks: list[Track]) -> np.ndarray:
    """Letterbox the sign camera into the fixed debug pane and draw tracks."""
    canvas, scale, pad_x, pad_y = letterbox(frame, DEBUG_WIDTH, DEBUG_HEIGHT)

    for track in tracks:
        x0, y0, x1, y1 = track.bbox
        p0 = (int(x0 * scale) + pad_x, int(y0 * scale) + pad_y)
        p1 = (int(x1 * scale) + pad_x, int(y1 * scale) + pad_y)

        if track.label == "traffic_light":
            colour = _LIGHT_COLOURS[track.light_state]
        else:
            colour = _BOX_COLOURS.get(track.label, (200, 200, 200))

        # Unconfirmed tracks are drawn thin: visible for debugging, but a
        # visual reminder that the policy is not acting on them yet.
        cv.rectangle(canvas, p0, p1, colour, 2 if track.confirmed else 1)

        caption = track.label
        if track.label == "traffic_light":
            caption = f"light:{track.light_state.value}"
        if track.distance_m is not None:
            caption += f" {track.distance_m:.1f}m"

        text_y = max(10, p0[1] - 4)
        cv.putText(canvas, caption, (p0[0], text_y), cv.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv.LINE_AA)

    return canvas


def _open_camera(index: int) -> Optional[cv.VideoCapture]:
    capture = cv.VideoCapture(index)
    if not capture.isOpened():
        capture.release()
        return None

    capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, _CAPTURE_WIDTH)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, _CAPTURE_HEIGHT)
    # Without this the driver queues frames and the detections drift steadily
    # further behind what the car is actually looking at.
    capture.set(cv.CAP_PROP_BUFFERSIZE, 1)
    capture.set(cv.CAP_PROP_AUTOFOCUS, 0)
    capture.set(cv.CAP_PROP_FOCUS, _FIXED_FOCUS)

    # Auto white balance is actively harmful for reading a traffic light, and
    # locking it is what makes colour behave the same in the test space as at
    # the venue. AWB assumes the scene has one illuminant and shifts everything
    # to make greys grey -- so a warm-lit room gets pushed blue, dragging a red
    # lamp toward magenta and a green one toward cyan. But a lamp is an
    # *emitter*: its colour comes from the LED, not from the room, and it was
    # already right before AWB touched it.
    #
    # Auto exposure is deliberately left on. It has the opposite trade: a fixed
    # exposure tuned for a bright room goes black in a dark one. The blown-out
    # lamp cores that auto exposure causes are already handled by reading hue
    # from the saturated halo rather than the centre.
    capture.set(cv.CAP_PROP_AUTO_WB, 0)

    logger.info(
        "Sign camera %s open at %dx%d",
        index,
        int(capture.get(cv.CAP_PROP_FRAME_WIDTH)),
        int(capture.get(cv.CAP_PROP_FRAME_HEIGHT)),
    )
    return capture
