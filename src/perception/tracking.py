"""Association of detections across frames into stable, confirmed tracks.

Three things come out of tracking that a per-frame detector cannot provide:

  Identity     A stop sign gets an ID that survives across frames, which is
               what lets the policy say "this one is already dealt with"
               rather than re-triggering on every frame it stays visible.

  Confirmation A single frame never decides anything. A track has to be seen
               repeatedly before the policy will act on it, and a committed
               light state needs more agreement to go green than to go red.

  Distance     Metres, from the bounding box height and the pinhole model,
               smoothed across frames. Metres are what braking is actually
               expressed in, and unlike a box-area fraction the number does
               not silently change meaning when the resolution does.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Optional
import math

from perception.sign_detection import Detection
from perception.traffic_light import LightState


# Nominal horizontal field of view of the OV5693 sign camera, degrees.
# tools/calibrate_sign_camera.py measures the real one.
SIGN_CAM_HFOV_DEG = 63.3

# Real height of each object in metres. Measure the *sign face only* and the
# *lamp housing only* -- never the post holding it up. Two reasons: the post
# length varies with the mount, which would make this not a constant; and the
# light reader splits the box into equal lamp bands, which only lines up if the
# box is the housing and nothing else. Training boxes must be drawn the same way.
#
# Placeholders until the signs are built. Distance stays monotonic meanwhile,
# which is enough to develop the behaviour against.
REAL_HEIGHT_M = {
    "stop": 0.43,
    "detour": 0.43,
    "traffic_light": 0.50,
    "pickup_a": 0.43,
    "pickup_b": 0.43,
    "pickup_c": 0.43,
}

# Boxes overlapping less than this are treated as different objects.
_MIN_IOU = 0.2

# A track unseen for longer than this is gone. Short, because acting on a
# stale box is how the previous implementation ended up halting on a light
# that had long since left the frame.
_MAX_TRACK_AGE_S = 0.5

# A track must appear in this many of the last few update cycles before the
# policy is allowed to act on it.
_CONFIRM_WINDOW = 5
_CONFIRM_HITS = 3

# Asymmetric hysteresis on the committed light state: two frames to start
# holding, four to release. Deliberately harder to go than to stop.
_LIGHT_WINDOW = 5
_LIGHT_HOLD_HITS = 2
_LIGHT_RELEASE_HITS = 4

# Box heights jitter frame to frame; the distance they imply should not.
_DISTANCE_EMA_ALPHA = 0.4


def focal_px_from_hfov(frame_width: int, hfov_deg: float = SIGN_CAM_HFOV_DEG) -> float:
    """Focal length in pixels implied by an image width and a field of view."""
    return (frame_width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def distance_from_height(real_height_m: float, focal_px: float, box_height_px: float) -> Optional[float]:
    """Pinhole distance to an object of known height."""
    if box_height_px <= 0 or focal_px <= 0:
        return None
    return real_height_m * focal_px / box_height_px


@dataclass
class Track:
    id: int
    label: str
    bbox: tuple[int, int, int, int]
    confidence: float
    distance_m: Optional[float]
    light_state: LightState
    light_confidence: float
    confirmed: bool
    first_seen: float
    last_seen: float


@dataclass
class _TrackState:
    """Mutable bookkeeping the policy never sees."""

    id: int
    label: str
    bbox: tuple[int, int, int, int]
    confidence: float
    first_seen: float
    last_seen: float
    distance_m: Optional[float] = None
    light_state: LightState = LightState.UNKNOWN
    light_confidence: float = 0.0
    hits: deque = field(default_factory=lambda: deque(maxlen=_CONFIRM_WINDOW))
    light_history: deque = field(default_factory=lambda: deque(maxlen=_LIGHT_WINDOW))

    @property
    def confirmed(self) -> bool:
        return sum(self.hits) >= _CONFIRM_HITS

    def snapshot(self) -> Track:
        return Track(
            id=self.id,
            label=self.label,
            bbox=self.bbox,
            confidence=self.confidence,
            distance_m=self.distance_m,
            light_state=self.light_state,
            light_confidence=self.light_confidence,
            confirmed=self.confirmed,
            first_seen=self.first_seen,
            last_seen=self.last_seen,
        )


class DetectionTracker:
    def __init__(self, hfov_deg: float = SIGN_CAM_HFOV_DEG):
        self._hfov_deg = hfov_deg
        self._tracks: list[_TrackState] = []
        self._next_id = 1

    def reset(self) -> None:
        # IDs deliberately keep climbing. The policy remembers which stop sign
        # it has already dealt with by track ID, so an ID must never be reused
        # for a different physical object.
        self._tracks.clear()

    def update(self, detections: list[Detection], frame_width: int, now: float) -> list[Track]:
        focal_px = focal_px_from_hfov(frame_width, self._hfov_deg)
        matched_tracks, matched_detections = self._associate(detections)

        for track_index, detection_index in matched_tracks.items():
            self._absorb(self._tracks[track_index], detections[detection_index], focal_px, now)

        for track_index, track in enumerate(self._tracks):
            if track_index not in matched_tracks:
                track.hits.append(0)

        for detection_index, detection in enumerate(detections):
            if detection_index not in matched_detections:
                self._tracks.append(self._spawn(detection, focal_px, now))

        self._tracks = [t for t in self._tracks if now - t.last_seen <= _MAX_TRACK_AGE_S]
        return [t.snapshot() for t in self._tracks]

    def _associate(self, detections: list[Detection]) -> tuple[dict, set]:
        """Greedy highest-IoU-first matching, restricted to matching labels."""
        candidates = []
        for track_index, track in enumerate(self._tracks):
            for detection_index, detection in enumerate(detections):
                if detection.label != track.label:
                    continue
                overlap = _iou(track.bbox, detection.bbox)
                if overlap >= _MIN_IOU:
                    candidates.append((overlap, track_index, detection_index))
        candidates.sort(reverse=True)

        matched_tracks: dict[int, int] = {}
        matched_detections: set[int] = set()
        for _, track_index, detection_index in candidates:
            if track_index in matched_tracks or detection_index in matched_detections:
                continue
            matched_tracks[track_index] = detection_index
            matched_detections.add(detection_index)
        return matched_tracks, matched_detections

    def _spawn(self, detection: Detection, focal_px: float, now: float) -> _TrackState:
        track = _TrackState(
            id=self._next_id,
            label=detection.label,
            bbox=detection.bbox,
            confidence=detection.confidence,
            first_seen=now,
            last_seen=now,
        )
        self._next_id += 1
        self._absorb(track, detection, focal_px, now, fresh=True)
        return track

    def _absorb(self, track: _TrackState, detection: Detection, focal_px: float, now: float, fresh: bool = False) -> None:
        track.bbox = detection.bbox
        track.confidence = detection.confidence
        track.last_seen = now
        track.hits.append(1)

        measured = self._measure_distance(detection, focal_px)
        if measured is not None:
            if track.distance_m is None or fresh:
                track.distance_m = measured
            else:
                track.distance_m = (
                    _DISTANCE_EMA_ALPHA * measured + (1 - _DISTANCE_EMA_ALPHA) * track.distance_m
                )

        if detection.label == "traffic_light":
            track.light_history.append(detection.light_state)
            track.light_confidence = detection.light_confidence
            self._commit_light_state(track)

    def _measure_distance(self, detection: Detection, focal_px: float) -> Optional[float]:
        real_height = REAL_HEIGHT_M.get(detection.label)
        if real_height is None:
            return None
        _, y0, _, y1 = detection.bbox
        return distance_from_height(real_height, focal_px, y1 - y0)

    def _commit_light_state(self, track: _TrackState) -> None:
        """Change the committed state only on enough agreement.

        Checked stop-first: if red has cleared its low bar, it wins even while
        greens are still arriving. UNKNOWN never commits — an unreadable light
        leaves the previous state standing, and the policy's maximum hold is
        what stops that from lasting forever.
        """
        recent = list(track.light_history)
        for state, needed in (
            (LightState.RED, _LIGHT_HOLD_HITS),
            (LightState.YELLOW, _LIGHT_HOLD_HITS),
            (LightState.GREEN, _LIGHT_RELEASE_HITS),
        ):
            if recent.count(state) >= needed:
                track.light_state = state
                return


def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b

    inter_w = min(ax1, bx1) - max(ax0, bx0)
    inter_h = min(ay1, by1) - max(ay0, by0)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0

    intersection = inter_w * inter_h
    union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - intersection
    return intersection / union if union > 0 else 0.0
