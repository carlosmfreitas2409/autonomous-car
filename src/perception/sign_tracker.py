"""Give detections an identity across frames, and only then an opinion.

The detector answers one frame at a time. The car needs more than that: the
same sign has to stay the same sign while it grows in the image, a box that
blinks out for two frames must not become a new object, a light's state must
not flip on one blown-out frame, and the policy downstream must be able to say
"I have dealt with *that one*" and mean it.

Everything here is pure Python over the detector's output: no camera, no
thread, no model. The two crop readers are injected so tests can fake them.
"""

from collections import Counter, deque
from dataclasses import dataclass
from typing import Callable, Optional
import math

import numpy as np

from perception import delivery_variant, traffic_light
from perception.delivery_variant import read_delivery_variant
from perception.sign_detection import Detection
from perception.traffic_light import LightState, read_light_state


# Physical heights of what the boxes bound: the printed sign face, or the lamp
# housing. Distance is real_height * focal / box_height, which only holds
# because every training box bounds exactly these objects and nothing else.
REAL_HEIGHT_M = {"stop": 0.43, "detour": 0.43, "delivery_sign": 0.43, "traffic_light": 0.50}

# The venue clips' USB camera, a 16:9 figure.
SIGN_CAM_HFOV_DEG = 63.3

# Red stops the car and waits for green. Yellow only slows it: it keeps
# rolling at the crawl floor, and if the light goes red while the car can
# still stop, the red gates it.
HALT_STATES = (LightState.RED,)
CAUTION_STATES = (LightState.YELLOW,)

# Association: same label, greedy highest IoU first.
MIN_IOU = 0.2

# Confirmation and hysteresis window: a track is confirmed when it was seen in
# CONFIRM_HITS of the last HISTORY updates, and a state or letter commits when
# it has CONFIRM_HITS of the last HISTORY readings.
HISTORY = 5
CONFIRM_HITS = 3

# A stop sign confirms faster: 2 of the last 3. The detector's measured
# precision on that class carries it, and a curve can reveal the sign with
# only a few frames of road left. Lights keep 3 of 5: their false positives
# live in the state reader, and the state itself still needs 3 of 5.
FAST_CONFIRM_LABELS = ("stop",)
FAST_HISTORY = 3
FAST_CONFIRM_HITS = 2

# A track unseen this long is dropped. Flicker shorter than this keeps its id.
EXPIRE_S = 0.6

DISTANCE_EMA_ALPHA = 0.4


@dataclass(frozen=True)
class Track:
    id: int
    label: str
    bbox: tuple[int, int, int, int]
    confidence: float
    distance_m: Optional[float]
    confirmed: bool
    light_state: LightState
    light_reason: str
    rejected: bool
    variant: Optional[str]
    variant_reason: str
    first_seen: float
    last_seen: float
    # Horizontal angle of the box centre off the camera axis, + right. For
    # display (the dashboard's world view); nothing decides on it.
    bearing_deg: Optional[float] = None

    @property
    def actionable(self) -> bool:
        """A light is only a light once it has a committed state."""
        if not self.confirmed:
            return False
        return self.label != "traffic_light" or self.light_state is not LightState.UNKNOWN


class _Record:
    """The tracker's mutable view of one object. Never leaves this module."""

    def __init__(self, track_id: int, detection: Detection, now: float):
        self.id = track_id
        self.label = detection.label
        self.bbox = detection.bbox
        self.confidence = detection.confidence
        self.first_seen = now
        self.last_seen = now
        self.hits: deque = deque(maxlen=HISTORY)
        self.distance: Optional[float] = None
        self.bearing: Optional[float] = None
        self.light_history: deque = deque(maxlen=HISTORY)
        self.light_state = LightState.UNKNOWN
        self.light_reason = ""
        self.rejected = False
        self.variant_history: deque = deque(maxlen=HISTORY)
        self.variant: Optional[str] = None
        self.variant_reason = ""

    @property
    def confirmed(self) -> bool:
        if sum(self.hits) >= CONFIRM_HITS:
            return True
        if self.label in FAST_CONFIRM_LABELS:
            recent = list(self.hits)[-FAST_HISTORY:]
            return sum(recent) >= FAST_CONFIRM_HITS
        return False

    def snapshot(self) -> Track:
        return Track(
            id=self.id, label=self.label, bbox=self.bbox, confidence=self.confidence,
            distance_m=self.distance, confirmed=self.confirmed,
            light_state=self.light_state, light_reason=self.light_reason, rejected=self.rejected,
            variant=self.variant, variant_reason=self.variant_reason,
            first_seen=self.first_seen, last_seen=self.last_seen, bearing_deg=self.bearing,
        )


class SignTracker:
    def __init__(self, hfov_deg: float = SIGN_CAM_HFOV_DEG,
                 read_light: Callable = read_light_state,
                 read_variant: Callable = read_delivery_variant):
        self._hfov = math.radians(hfov_deg)
        self._read_light = read_light
        self._read_variant = read_variant
        self._records: list[_Record] = []
        self._next_id = 1

    def reset(self) -> None:
        """Forget every track. Ids keep climbing: the policy consumes by id and
        an id that has gone must never come back."""
        self._records = []

    def update(self, detections: list[Detection], frame: np.ndarray, now: float) -> list[Track]:
        self._records = [r for r in self._records if now - r.last_seen <= EXPIRE_S]

        matched_records: set[int] = set()
        matched_detections: set[int] = set()
        for record_index, detection_index in self._associate(detections):
            matched_records.add(record_index)
            matched_detections.add(detection_index)
            self._observe(self._records[record_index], detections[detection_index], frame, now)

        for index, record in enumerate(self._records):
            if index not in matched_records:
                record.hits.append(0)

        for index, detection in enumerate(detections):
            if index in matched_detections:
                continue
            record = _Record(self._next_id, detection, now)
            self._next_id += 1
            self._records.append(record)
            self._observe(record, detection, frame, now)

        return [record.snapshot() for record in self._records]

    # ------------------------------------------------------------------

    def _associate(self, detections: list[Detection]) -> list[tuple[int, int]]:
        pairs = []
        for r_index, record in enumerate(self._records):
            for d_index, detection in enumerate(detections):
                if detection.label != record.label:
                    continue
                overlap = _iou(record.bbox, detection.bbox)
                if overlap >= MIN_IOU:
                    pairs.append((overlap, r_index, d_index))
        pairs.sort(reverse=True)

        used_records: set[int] = set()
        used_detections: set[int] = set()
        matches = []
        for _, r_index, d_index in pairs:
            if r_index in used_records or d_index in used_detections:
                continue
            used_records.add(r_index)
            used_detections.add(d_index)
            matches.append((r_index, d_index))
        return matches

    def _observe(self, record: _Record, detection: Detection, frame: np.ndarray, now: float) -> None:
        record.bbox = detection.bbox
        record.confidence = detection.confidence
        record.last_seen = now
        record.hits.append(1)
        self._update_distance(record, detection, frame)
        x0, _, x1, _ = detection.bbox
        record.bearing = bearing_deg((x0 + x1) / 2.0, frame.shape[1], self._hfov)

        if record.label == "traffic_light":
            reading = self._read_light(detection.crop(frame, traffic_light.CROP_MARGIN))
            self._commit_light(record, reading)
        elif record.label == "delivery_sign":
            reading = self._read_variant(detection.crop(frame, delivery_variant.CROP_MARGIN))
            self._commit_variant(record, reading)

    def _update_distance(self, record: _Record, detection: Detection, frame: np.ndarray) -> None:
        frame_h, frame_w = frame.shape[:2]
        x0, y0, x1, y1 = detection.bbox
        # A box on the top or bottom edge is clipped: it reads too far.
        if y0 <= 0 or y1 >= frame_h - 1 or detection.height <= 0:
            return
        focal_px = (frame_w / 2.0) / math.tan(self._hfov / 2.0)
        raw = REAL_HEIGHT_M[record.label] * focal_px / detection.height
        if record.distance is None:
            record.distance = raw
        else:
            record.distance = DISTANCE_EMA_ALPHA * raw + (1 - DISTANCE_EMA_ALPHA) * record.distance

    def _commit_light(self, record: _Record, reading) -> None:
        record.light_history.append(reading.state)
        record.rejected = not reading.verified
        # UNKNOWN never commits and never votes against a committed state.
        votes = Counter(s for s in record.light_history if s is not LightState.UNKNOWN)
        for state, count in votes.most_common(1):
            if count >= CONFIRM_HITS:
                record.light_state = state
        if record.rejected:
            record.light_reason = f"rejected: {reading.reason}"
        elif reading.state is LightState.UNKNOWN:
            record.light_reason = reading.reason or "unknown"
        else:
            record.light_reason = reading.reason

    def _commit_variant(self, record: _Record, reading) -> None:
        record.variant_history.append(reading.variant)
        votes = Counter(v for v in record.variant_history if v is not None)
        committed = None
        for letter, count in votes.most_common(1):
            if count >= CONFIRM_HITS:
                committed = letter
        record.variant = committed
        record.variant_reason = reading.reason


def bearing_deg(center_x: float, frame_w: int, hfov_rad: float) -> float:
    """The angle of an image column off the optical axis, + right, with the
    same pinhole focal length the distance uses."""
    focal_px = (frame_w / 2.0) / math.tan(hfov_rad / 2.0)
    return math.degrees(math.atan((center_x - frame_w / 2.0) / focal_px))


def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    inter_w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = inter_w * inter_h
    if intersection == 0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0
