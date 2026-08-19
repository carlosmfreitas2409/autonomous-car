"""The glue between detection, light reading, and the debug pane.

The cropping in particular is safety-relevant: read the wrong pixels and the
light state is wrong, however good the reader is.
"""

import time

import numpy as np

from perception.sign_detection import Detection
from perception.sign_worker import (
    DEBUG_HEIGHT,
    DEBUG_WIDTH,
    SignSnapshot,
    SignWorker,
    _read_light_states,
    render_debug_frame,
)
from perception.tracking import Track
from perception.traffic_light import LightState
from synthetic import render_light


def frame_with_light(state, x=200, y=100, lamp_px=40):
    """A dark scene with one rendered traffic light pasted into it."""
    frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
    light = render_light(state, lamp_px=lamp_px)
    h, w = light.shape[:2]
    frame[y:y + h, x:x + w] = light
    return frame, (x, y, x + w, y + h)


def test_light_state_is_read_from_the_boxed_crop():
    frame, bbox = frame_with_light(LightState.RED)
    detection = Detection(label="traffic_light", bbox=bbox, confidence=0.9)

    _read_light_states(frame, [detection])

    assert detection.light_state is LightState.RED
    assert detection.light_confidence > 0.0


def test_each_light_is_read_from_its_own_box():
    frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
    detections = []
    for i, state in enumerate((LightState.RED, LightState.GREEN)):
        light = render_light(state)
        h, w = light.shape[:2]
        x = 200 + i * 400
        frame[100:100 + h, x:x + w] = light
        detections.append(Detection(label="traffic_light", bbox=(x, 100, x + w, 100 + h), confidence=0.9))

    _read_light_states(frame, detections)

    assert [d.light_state for d in detections] == [LightState.RED, LightState.GREEN]


def test_non_light_detections_are_left_alone():
    frame, bbox = frame_with_light(LightState.RED)
    detection = Detection(label="stop", bbox=bbox, confidence=0.9)

    _read_light_states(frame, [detection])

    assert detection.light_state is LightState.UNKNOWN


def test_a_box_running_off_the_frame_does_not_crash():
    frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
    detections = [
        Detection(label="traffic_light", bbox=(-50, -50, 60, 130), confidence=0.9),
        Detection(label="traffic_light", bbox=(1240, 690, 1400, 900), confidence=0.9),
    ]

    _read_light_states(frame, detections)

    assert all(d.light_state is LightState.UNKNOWN for d in detections)


def test_debug_frame_is_always_the_pane_size():
    for shape in ((720, 1280, 3), (480, 640, 3), (1944, 2592, 3)):
        canvas = render_debug_frame(np.zeros(shape, dtype=np.uint8), [])
        assert canvas.shape == (DEBUG_HEIGHT, DEBUG_WIDTH, 3)


def test_debug_frame_draws_tracks():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    tracks = [
        Track(1, "stop", (100, 100, 200, 200), 0.9, 1.5, LightState.UNKNOWN, 0.0, True, 0.0, 0.0),
        Track(2, "traffic_light", (400, 100, 440, 220), 0.9, 2.5, LightState.GREEN, 0.9, True, 0.0, 0.0),
    ]

    blank = render_debug_frame(frame, [])
    drawn = render_debug_frame(frame, tracks)

    assert not np.array_equal(blank, drawn)


def test_a_snapshot_that_never_published_is_infinitely_old():
    assert SignSnapshot().age(100.0) == float("inf")


def test_snapshot_age_is_measured_from_its_timestamp():
    assert SignSnapshot(timestamp=99.5).age(100.0) == 0.5


def wait_for_first_snapshot(worker, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = worker.get_snapshot()
        if snapshot.timestamp != 0.0:
            return snapshot
        time.sleep(0.02)
    return worker.get_snapshot()


def test_a_missing_sign_camera_never_reaches_the_control_loop():
    """No camera and no model must degrade to "nothing detected", not to an
    exception or a stalled thread. The control loop has a 500 ms watchdog."""
    worker = SignWorker(camera_index=999, model_path="does-not-exist.onnx")
    worker.start()
    try:
        snapshot = wait_for_first_snapshot(worker)
    finally:
        worker.close()

    assert snapshot.camera_open is False
    assert snapshot.detector_available is False
    assert snapshot.tracks == []


def test_reading_a_snapshot_is_cheap_enough_for_the_control_loop():
    """This is the whole per-iteration cost of sign detection on the drive
    side, so it needs to stay far below the 500 ms watchdog budget."""
    worker = SignWorker(camera_index=999, model_path="does-not-exist.onnx")
    worker.start()
    try:
        wait_for_first_snapshot(worker)
        start = time.perf_counter()
        for _ in range(1000):
            worker.get_snapshot()
        per_call_ms = (time.perf_counter() - start) * 1000 / 1000
    finally:
        worker.close()

    assert per_call_ms < 0.5


def test_close_is_idempotent_and_returns_promptly():
    worker = SignWorker(camera_index=999, model_path="does-not-exist.onnx")
    worker.start()
    wait_for_first_snapshot(worker)

    start = time.monotonic()
    worker.close()
    worker.close()
    assert time.monotonic() - start < 2.0
