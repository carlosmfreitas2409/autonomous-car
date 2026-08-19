"""Letterboxing, YOLO output decoding, and NMS.

The ONNX session itself is plumbing, but the geometry around it is not: a sign
detected at the wrong place is a sign braked for at the wrong distance. These
drive the decode path with hand-built model output, so they need no weights.
"""

import numpy as np
import pytest

from perception.sign_detection import SignDetector, _nms, letterbox, plausible_shape


FRAME_W, FRAME_H = 1280, 720
MODEL_SIZE = 640


def detector_with(labels: dict[int, str], confidence: float = 0.5) -> SignDetector:
    """A detector with no session, used purely to exercise decoding."""
    detector = SignDetector("does-not-exist.onnx", confidence=confidence)
    assert detector.available is False
    detector._labels = labels
    return detector


def model_output(rows: list[list[float]]) -> np.ndarray:
    """Shape (1, 4 + num_classes, num_boxes), as ultralytics emits."""
    return np.array(rows, dtype=np.float32).T[np.newaxis]


def test_letterbox_preserves_aspect_and_centres():
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    canvas, scale, pad_x, pad_y = letterbox(frame, MODEL_SIZE, MODEL_SIZE)

    assert canvas.shape == (MODEL_SIZE, MODEL_SIZE, 3)
    assert scale == pytest.approx(0.5)
    assert (pad_x, pad_y) == (0, 140)


def test_letterbox_into_a_non_square_pane():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    canvas, scale, pad_x, pad_y = letterbox(frame, 640, 360)

    assert canvas.shape == (360, 640, 3)
    assert scale == pytest.approx(0.75)
    assert (pad_x, pad_y) == (80, 0)


def test_decode_maps_a_box_back_to_source_pixels():
    detector = detector_with({0: "stop"})
    # A source box of (100, 200)-(200, 300) after a 0.5 scale and 140 px of
    # vertical padding sits at centre (75, 265), size 50x50.
    detections = detector._decode(
        model_output([[75, 265, 50, 50, 0.9]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    )

    assert len(detections) == 1
    assert detections[0].label == "stop"
    assert detections[0].bbox == (100, 200, 200, 300)
    assert detections[0].confidence == pytest.approx(0.9)


def test_decode_applies_the_coco_label_aliases():
    """The bootstrap COCO model names these differently to the custom one."""
    detector = detector_with({0: "stop sign", 1: "traffic light"})
    detections = detector._decode(
        model_output([[75, 265, 50, 50, 0.9, 0.0], [215, 228, 30, 75, 0.0, 0.8]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    )
    assert sorted(d.label for d in detections) == ["stop", "traffic_light"]


def test_decode_drops_classes_the_behaviour_layer_cannot_use():
    detector = detector_with({0: "stop", 1: "person"})
    detections = detector._decode(
        model_output([[75, 265, 50, 50, 0.9, 0.0], [300, 300, 40, 80, 0.0, 0.95]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    )
    assert [d.label for d in detections] == ["stop"]


def test_decode_respects_the_confidence_threshold():
    detector = detector_with({0: "stop"}, confidence=0.5)
    detections = detector._decode(
        model_output([[75, 265, 50, 50, 0.3]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    )
    assert detections == []


def test_decode_clips_boxes_to_the_frame():
    detector = detector_with({0: "stop"})
    detections = detector._decode(
        model_output([[10, 150, 60, 60, 0.9]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    )
    x0, y0, x1, y1 = detections[0].bbox
    assert 0 <= x0 < x1 <= FRAME_W - 1
    assert 0 <= y0 < y1 <= FRAME_H - 1


def test_decode_handles_an_empty_prediction_set():
    detector = detector_with({0: "stop"})
    assert detector._decode(
        model_output([[10, 10, 4, 4, 0.01]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    ) == []


def test_nms_suppresses_a_duplicate_box():
    boxes = np.array([[100, 100, 200, 200], [104, 104, 204, 204]], dtype=np.float32)
    keep = _nms(boxes, np.array([0.9, 0.8], dtype=np.float32), 0.45)
    assert keep == [0]


def test_nms_keeps_separate_objects():
    boxes = np.array([[100, 100, 200, 200], [400, 400, 500, 500]], dtype=np.float32)
    keep = _nms(boxes, np.array([0.9, 0.8], dtype=np.float32), 0.45)
    assert sorted(keep) == [0, 1]


def test_nms_returns_boxes_strongest_first():
    boxes = np.array([[0, 0, 10, 10], [400, 400, 500, 500], [800, 400, 900, 500]], dtype=np.float32)
    keep = _nms(boxes, np.array([0.3, 0.9, 0.6], dtype=np.float32), 0.45)
    assert keep == [1, 2, 0]


def test_detect_returns_nothing_without_a_model():
    detector = SignDetector("does-not-exist.onnx")
    assert detector.detect(np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)) == []


def test_a_person_shaped_box_is_not_a_traffic_light():
    """The commonest false positive: a standing person is roughly 4:1 and
    cannot be a 2.2:1 housing, whatever confidence the model reports."""
    assert plausible_shape("traffic_light", width=60, height=240) is False
    assert plausible_shape("traffic_light", width=60, height=132) is True


def test_a_wide_rectangle_is_not_a_square_sign():
    """A phone screen, a doorway, a strip of text."""
    assert plausible_shape("stop", width=200, height=60) is False
    assert plausible_shape("pickup_a", width=60, height=200) is False
    assert plausible_shape("pickup_a", width=100, height=95) is True


def test_shape_gate_allows_for_viewing_angle():
    """A flat sign seen off-axis narrows; one seen from below shortens."""
    for height in (70, 100, 140):
        assert plausible_shape("detour", width=100, height=height) is True


def test_unknown_labels_are_not_shape_gated():
    assert plausible_shape("something_else", width=500, height=1) is True


def test_decode_drops_implausibly_shaped_boxes():
    detector = detector_with({0: "traffic_light"})
    detections = detector._decode(
        # 40 px wide, 320 px tall in source pixels: person-shaped.
        model_output([[75, 290, 20, 160, 0.95]]),
        scale=0.5, pad_x=0, pad_y=140, frame_shape=(FRAME_H, FRAME_W),
    )
    assert detections == []
