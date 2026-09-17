"""The ONNX wrapper's arithmetic and its behaviour with no model at all.

A fresh clone has no models/signs.onnx - training happens on another machine -
so "the detector is missing" has to be an ordinary state, not a crash.
"""

import numpy as np
import pytest

from perception.sign_detection import (INPUT_SIZE, KNOWN_LABELS, Detection, SignDetector,
                                       letterbox, non_max_suppression, xywh_to_xyxy)


def test_missing_model_is_not_an_error():
    detector = SignDetector("models/definitely-not-here.onnx")
    assert not detector.available
    assert detector.detect(np.zeros((480, 640, 3), np.uint8)) == []


def test_class_list_matches_the_compositor():
    """tools/synthesize_dataset.py writes class indices in this order and bakes
    the names into the ONNX metadata; a mismatch relabels every detection."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
    from synthesize_dataset import CLASSES
    assert tuple(CLASSES) == KNOWN_LABELS


@pytest.mark.parametrize("shape", [(720, 1280), (480, 640), (1080, 1080), (200, 900)])
def test_letterbox_round_trips_a_point(shape):
    frame = np.zeros((*shape, 3), np.uint8)
    canvas, scale, pad_x, pad_y = letterbox(frame, INPUT_SIZE, INPUT_SIZE)
    assert canvas.shape == (INPUT_SIZE, INPUT_SIZE, 3)
    for point in ((0, 0), (shape[1] - 1, shape[0] - 1), (shape[1] // 3, shape[0] // 4)):
        mapped = (point[0] * scale + pad_x, point[1] * scale + pad_y)
        back = ((mapped[0] - pad_x) / scale, (mapped[1] - pad_y) / scale)
        assert back == pytest.approx(point, abs=1e-6)


def test_letterbox_does_not_distort():
    canvas, scale, pad_x, pad_y = letterbox(np.zeros((360, 1280, 3), np.uint8), 640, 640)
    assert pad_x == 0
    assert pad_y > 0


def test_xywh_to_xyxy():
    boxes = xywh_to_xyxy(np.array([[100.0, 50.0, 20.0, 10.0]]))
    assert boxes[0] == pytest.approx([90, 45, 110, 55])


def test_nms_drops_the_overlapping_box():
    boxes = np.array([[0.0, 0.0, 10.0, 10.0],
                      [1.0, 1.0, 11.0, 11.0],
                      [50.0, 50.0, 60.0, 60.0]])
    kept = non_max_suppression(boxes, np.array([0.9, 0.8, 0.7]), 0.45)
    assert sorted(kept) == [0, 2]


def test_nms_keeps_neighbours_that_barely_touch():
    boxes = np.array([[0.0, 0.0, 10.0, 10.0], [9.0, 0.0, 19.0, 10.0]])
    assert len(non_max_suppression(boxes, np.array([0.9, 0.8]), 0.45)) == 2


def test_detection_crop_is_the_box():
    frame = np.arange(100 * 120 * 3, dtype=np.uint8).reshape(100, 120, 3)
    detection = Detection("stop", (10, 20, 40, 60), 0.9)
    assert detection.width == 30 and detection.height == 40
    assert detection.crop(frame).shape == (40, 30, 3)


def test_detection_crop_with_margin_clamps_to_the_frame():
    frame = np.zeros((100, 120, 3), np.uint8)
    detection = Detection("traffic_light", (2, 2, 40, 60), 0.9)
    crop = detection.crop(frame, margin=0.5)
    assert crop.shape[0] <= 100 and crop.shape[1] <= 120
    assert crop.size > 0
