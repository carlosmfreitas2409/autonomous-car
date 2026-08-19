"""The whole chain, from pixels to a throttle ceiling.

Each piece is unit tested elsewhere. What this covers is that they agree with
each other: that the light reader's answer survives the tracker's hysteresis,
that the tracker's distance is in the units the policy expects, and that an
approach actually produces braking at the right point.

The only thing stubbed is the detector, which supplies the bounding box a
trained model would. Everything downstream of that box is the real code.
"""

import numpy as np
import pytest

from input.traffic_policy import PolicyState, TrafficPolicy
from perception.sign_detection import Detection
from perception.sign_worker import SignSnapshot, _read_light_states
from perception.tracking import DetectionTracker, REAL_HEIGHT_M, focal_px_from_hfov
from perception.traffic_light import LightState
from synthetic import render_light


FRAME_W = 1280
# Taller than a real frame on purpose. Box sizes here are derived from the
# project's actual measured sign heights, and at half a metre a 50 cm light is
# over a thousand pixels tall — it would be clipped by a 720 px canvas and the
# bottom lamp cut off. The extra height keeps the scene builder honest; FRAME_W
# is what sets the focal length, so the distances are unaffected.
FRAME_H = 1600
DT = 0.1
THROTTLE = 0.5


def box_height_for(distance_m: float, label: str = "traffic_light") -> int:
    """Inverse of the tracker's pinhole distance, so a scene can be built for
    a chosen distance."""
    focal = focal_px_from_hfov(FRAME_W)
    return int(round(REAL_HEIGHT_M[label] * focal / distance_m))


def scene_with_light(state: LightState, distance_m: float):
    """A frame containing a traffic light rendered at the right apparent size."""
    lamp_px = max(4, box_height_for(distance_m) // 3)
    light = render_light(state, lamp_px=lamp_px)
    height, width = light.shape[:2]

    frame = np.full((FRAME_H, FRAME_W, 3), 55, dtype=np.uint8)
    x, y = 600, max(0, (FRAME_H - height) // 3)
    height = min(height, FRAME_H - y)
    frame[y:y + height, x:x + width] = light[:height]
    return frame, (x, y, x + width, y + height)


def _sweep(start_m: float, end_m: float, step_m: float = 0.06) -> list[float]:
    """Distances from start to end at roughly one frame's worth of travel."""
    steps = max(6, int(abs(start_m - end_m) / step_m))
    return [float(d) for d in np.linspace(start_m, end_m, steps)]


class Rig:
    """Detector-shaped stub in front of the real tracker and policy."""

    def __init__(self):
        self.tracker = DetectionTracker()
        self.policy = TrafficPolicy()
        self.now = 100.0

    def step(self, frame, boxes, throttle=THROTTLE):
        detections = [Detection(label=label, bbox=bbox, confidence=0.9) for label, bbox in boxes]
        _read_light_states(frame, detections)

        self.now += DT
        tracks = self.tracker.update(detections, FRAME_W, self.now)
        snapshot = SignSnapshot(
            tracks=tracks, timestamp=self.now, camera_open=True, detector_available=True
        )
        return self.policy.update(snapshot, throttle, DT, self.now)

    def approach_light(self, state, start_m, end_m):
        """Close on a light gradually, the way the cart actually would.

        Step size matters: at a realistic frame rate the cart covers a few
        centimetres per frame, and the tracker's distance smoothing is tuned
        for that. Marching in half-metre jumps would fight it.
        """
        constraint = None
        for distance in _sweep(start_m, end_m):
            frame, bbox = scene_with_light(state, distance)
            constraint = self.step(frame, [("traffic_light", bbox)])
        return constraint

    def hold_light(self, state, distance_m, frames=6):
        frame, bbox = scene_with_light(state, distance_m)
        constraint = None
        for _ in range(frames):
            constraint = self.step(frame, [("traffic_light", bbox)], throttle=0.0)
        return constraint


@pytest.mark.parametrize("state", [LightState.RED, LightState.YELLOW, LightState.GREEN])
def test_rendered_light_is_read_correctly_at_realistic_sizes(state):
    """Sanity: the scene builder produces lights the reader can actually read."""
    for distance in (4.0, 2.0, 1.0, 0.5):
        frame, bbox = scene_with_light(state, distance)
        detection = Detection(label="traffic_light", bbox=bbox, confidence=0.9)
        _read_light_states(frame, [detection])
        assert detection.light_state is state, f"misread at {distance} m"


def test_a_red_light_brings_the_cart_to_a_stop():
    rig = Rig()

    far = rig.approach_light(LightState.RED, 8.0, 5.0)
    assert far.state is PolicyState.CLEAR
    assert far.speed_cap == 1.0

    slowing = rig.approach_light(LightState.RED, 5.0, 2.0)
    assert slowing.state is PolicyState.SLOWING
    assert slowing.speed_cap < 1.0

    closer = rig.approach_light(LightState.RED, 2.0, 1.2)
    assert closer.speed_cap < slowing.speed_cap

    stopped = rig.approach_light(LightState.RED, 1.2, 0.3)
    assert stopped.hold is True
    assert stopped.speed_cap == 0.0
    assert stopped.state is PolicyState.WAITING_GREEN


def test_the_cart_moves_off_once_the_light_turns_green():
    rig = Rig()
    assert rig.approach_light(LightState.RED, 4.0, 0.5).hold is True

    # Green takes more agreement than red did, so one frame is not enough.
    frame, bbox = scene_with_light(LightState.GREEN, 0.5)
    assert rig.step(frame, [("traffic_light", bbox)], throttle=0.0).hold is True

    released = rig.hold_light(LightState.GREEN, 0.5)
    assert released.hold is False
    assert released.state is PolicyState.CLEAR


def test_a_green_light_is_driven_straight_through():
    rig = Rig()
    constraint = rig.approach_light(LightState.GREEN, 5.0, 0.5)
    assert constraint.hold is False
    assert constraint.speed_cap == 1.0


def test_a_stop_sign_stops_dwells_and_releases():
    rig = Rig()
    frame = np.full((FRAME_H, FRAME_W, 3), 55, dtype=np.uint8)

    def sign_at(distance_m):
        height = box_height_for(distance_m, "stop")
        return ("stop", (400, 200, 400 + height, 200 + height))

    def approach(start_m, end_m, throttle=THROTTLE):
        constraint = None
        for distance in _sweep(start_m, end_m):
            constraint = rig.step(frame, [sign_at(distance)], throttle=throttle)
        return constraint

    assert approach(8.0, 5.0).state is PolicyState.CLEAR
    assert approach(5.0, 2.0).state is PolicyState.SLOWING

    stopped = approach(2.0, 0.5)
    assert stopped.hold is True
    assert stopped.state in (PolicyState.HOLDING, PolicyState.DWELLING)

    # Sitting at the line through the settle and the dwell.
    for _ in range(30):
        assert rig.step(frame, [sign_at(0.5)], throttle=0.0).hold is True

    for _ in range(10):
        rig.step(frame, [sign_at(0.5)], throttle=0.0)
    assert rig.step(frame, [sign_at(0.5)], throttle=0.0).hold is False

    # And it never stops again at the same sign.
    for _ in range(20):
        assert rig.step(frame, [sign_at(0.5)], throttle=0.0).hold is False


def test_perception_going_dark_releases_a_light_the_car_is_waiting_at():
    """A gate that only opens on seeing green cannot stay shut once frames stop
    arriving - it could never reopen, so it would just stall until the maximum
    hold expired."""
    rig = Rig()
    assert rig.approach_light(LightState.RED, 4.0, 0.5).hold is True

    stale = SignSnapshot(tracks=[], timestamp=rig.now - 5.0, camera_open=False)
    constraint = rig.policy.update(stale, 0.0, DT, rig.now)

    assert constraint.stale is True
    assert constraint.speed_cap == 1.0
    assert constraint.hold is False


def test_perception_going_dark_still_completes_a_committed_stop():
    """The opposite case, and deliberately so: a stop-sign latch is bounded and
    self-terminating, so losing the camera mid-approach still stops the car."""
    rig = Rig()
    frame = np.full((FRAME_H, FRAME_W, 3), 55, dtype=np.uint8)
    height = box_height_for(1.0, "stop")

    # Enough frames for the track to confirm and the latch to commit, then the
    # camera dies.
    for _ in range(4):
        rig.step(frame, [("stop", (400, 200, 400 + height, 200 + height))])

    stale = SignSnapshot(tracks=[], timestamp=rig.now - 5.0, camera_open=False)
    constraint = None
    for _ in range(40):
        rig.now += DT
        constraint = rig.policy.update(stale, THROTTLE, DT, rig.now)
        if constraint.hold:
            break

    assert constraint.hold is True
