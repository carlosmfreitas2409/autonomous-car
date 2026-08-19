"""YOLO sign detection through onnxruntime.

Runtime is deliberately ONNX rather than ultralytics: ~30 MB against ~2.5 GB,
no torch import on startup, and the same artefact runs unchanged if this moves
off the laptop onto a Pi. Training and export stay offline in tools/.

The detector is synchronous and owns no camera and no thread, so it can be
called directly from a test or an offline script. SignWorker is what puts it
on a thread.

If the model file or onnxruntime is missing, the detector reports itself
unavailable and returns nothing. That is what keeps the car drivable from a
fresh clone with no model trained and no new dependency installed.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import ast
import logging
import cv2 as cv
import numpy as np

from perception.traffic_light import LightState

logger = logging.getLogger(__name__)


# Square input the model was exported at. Small enough to stay well inside the
# frame budget on a laptop, large enough that a 15 cm sign at 3 m still covers
# the ~20 px a detector needs.
_INPUT_SIZE = 640

# Grey letterbox padding, the value ultralytics trains against.
_PAD_VALUE = 114

_NMS_IOU = 0.45

# Everything the behaviour layer knows how to act on. Anything else the model
# emits is dropped here rather than travelling further, so a class trained into
# the model but missing from this tuple disappears silently. These names must
# match the directory names under assets/sign_cutouts/.
KNOWN_LABELS = ("stop", "detour", "traffic_light", "pickup_a", "pickup_b", "pickup_c")

# Signs that are reported to the routing layer but carry no braking behaviour
# of their own.
INFORMATIONAL_LABELS = ("pickup_a", "pickup_b", "pickup_c")

# Plausible box shape (height / width) per class, measured off the artwork with
# room for viewing angle. A box outside its class's range is not that sign,
# whatever confidence the model reports — a standing person is roughly 4:1 and
# cannot be a 2.2:1 light housing, and a phone screen is not a square sign.
#
# This is a shape check, not a tuning knob: the numbers come from the physical
# signs and change only if the signs do.
PLAUSIBLE_ASPECT = {
    "stop": (0.65, 1.55),
    "detour": (0.65, 1.55),
    "pickup_a": (0.65, 1.55),
    "pickup_b": (0.65, 1.55),
    "pickup_c": (0.65, 1.55),
    "traffic_light": (1.50, 3.30),
}

# The bootstrap COCO model names two of our classes differently. Mapping them
# here means the stock model and the custom one take the same code path.
_LABEL_ALIASES = {
    "stop sign": "stop",
    "traffic light": "traffic_light",
}

_PREFERRED_PROVIDERS = ("CoreMLExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider")


@dataclass
class Detection:
    label: str
    bbox: tuple[int, int, int, int]
    confidence: float
    # Filled in by SignWorker for traffic lights only, by reading the crop.
    # The model itself is deliberately state-agnostic.
    light_state: LightState = LightState.UNKNOWN
    light_confidence: float = 0.0


class SignDetector:
    def __init__(self, model_path: str, confidence: float = 0.45):
        self._confidence = confidence
        self._session = None
        self._input_name: Optional[str] = None
        self._input_size = _INPUT_SIZE
        self._labels: dict[int, str] = {}
        self._load(Path(model_path))

    @property
    def available(self) -> bool:
        return self._session is not None

    def set_confidence(self, confidence: float) -> None:
        self._confidence = confidence

    def detect(self, frame: np.ndarray) -> list[Detection]:
        if self._session is None or frame is None or frame.size == 0:
            return []

        blob, scale, pad_x, pad_y = self._preprocess(frame)
        outputs = self._session.run(None, {self._input_name: blob})
        return self._decode(outputs[0], scale, pad_x, pad_y, frame.shape[:2])

    def _load(self, model_path: Path) -> None:
        try:
            import onnxruntime as ort
        except ImportError:
            logger.warning("onnxruntime not installed - sign detection disabled")
            return

        if not model_path.exists():
            logger.warning("No sign model at %s - sign detection disabled", model_path)
            return

        providers = [p for p in _PREFERRED_PROVIDERS if p in ort.get_available_providers()]
        try:
            session = ort.InferenceSession(str(model_path), providers=providers)
        except Exception:
            logger.warning("Falling back to CPU for sign detection", exc_info=True)
            try:
                session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            except Exception:
                logger.error("Could not load sign model %s - sign detection disabled", model_path, exc_info=True)
                return

        self._session = session
        model_input = session.get_inputs()[0]
        self._input_name = model_input.name
        # A fixed-shape export pins the input size; a dynamic one leaves it to us.
        if isinstance(model_input.shape[2], int):
            self._input_size = model_input.shape[2]
        self._labels = _read_labels(session, model_path)
        logger.info(
            "Sign model %s loaded (%dpx, %s), classes: %s",
            model_path.name, self._input_size, session.get_providers()[0],
            sorted(set(self._labels.values()) & set(KNOWN_LABELS)),
        )

    def _preprocess(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        canvas, scale, pad_x, pad_y = letterbox(frame, self._input_size, self._input_size)
        rgb = cv.cvtColor(canvas, cv.COLOR_BGR2RGB)
        blob = rgb.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
        return np.ascontiguousarray(blob), scale, pad_x, pad_y

    def _decode(self, output: np.ndarray, scale: float, pad_x: int, pad_y: int, frame_shape) -> list[Detection]:
        # Ultralytics emits (1, 4 + num_classes, num_boxes).
        predictions = output[0].T
        if predictions.shape[1] <= 4:
            return []

        class_scores = predictions[:, 4:]
        class_ids = class_scores.argmax(axis=1)
        confidences = class_scores[np.arange(class_scores.shape[0]), class_ids]

        keep = confidences >= self._confidence
        if not keep.any():
            return []

        boxes = _xywh_to_xyxy(predictions[keep, :4])
        confidences = confidences[keep]
        class_ids = class_ids[keep]

        # Undo the letterbox so boxes land back in source-frame pixels.
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / scale

        height, width = frame_shape
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, width - 1)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, height - 1)

        detections = []
        for index in _nms(boxes, confidences, _NMS_IOU):
            label = self._labels.get(int(class_ids[index]))
            label = _LABEL_ALIASES.get(label, label)
            if label not in KNOWN_LABELS:
                continue
            x0, y0, x1, y1 = boxes[index].astype(int)
            if x1 <= x0 or y1 <= y0:
                continue
            if not plausible_shape(label, x1 - x0, y1 - y0):
                continue
            detections.append(
                Detection(label=label, bbox=(int(x0), int(y0), int(x1), int(y1)), confidence=float(confidences[index]))
            )
        return detections


def plausible_shape(label: str, width: float, height: float) -> bool:
    """Could a box this shape be this sign at all?"""
    bounds = PLAUSIBLE_ASPECT.get(label)
    if bounds is None or width <= 0:
        return True
    low, high = bounds
    return low <= height / width <= high


def letterbox(frame: np.ndarray, width: int, height: int, pad_value: int = _PAD_VALUE):
    """Fit a frame into a canvas without distorting it.

    Returns the canvas plus the scale and offsets needed to map coordinates
    back and forth, so both the model input and the debug view can use it.
    """
    source_h, source_w = frame.shape[:2]
    scale = min(width / source_w, height / source_h)
    new_w, new_h = max(1, int(round(source_w * scale))), max(1, int(round(source_h * scale)))

    canvas = np.full((height, width, 3), pad_value, dtype=np.uint8)
    pad_x, pad_y = (width - new_w) // 2, (height - new_h) // 2
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv.resize(frame, (new_w, new_h))
    return canvas, scale, pad_x, pad_y


def _read_labels(session, model_path: Path) -> dict[int, str]:
    """Class names from the ONNX metadata, or a sidecar .names file."""
    metadata = session.get_modelmeta().custom_metadata_map or {}
    if "names" in metadata:
        try:
            return {int(k): str(v) for k, v in ast.literal_eval(metadata["names"]).items()}
        except (ValueError, SyntaxError, AttributeError):
            logger.warning("Could not parse class names embedded in %s", model_path.name)

    sidecar = model_path.with_suffix(".names")
    if sidecar.exists():
        names = [line.strip() for line in sidecar.read_text().splitlines() if line.strip()]
        return dict(enumerate(names))

    logger.warning("No class names found for %s - detections will be dropped", model_path.name)
    return {}


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    converted = np.empty_like(boxes)
    half_w, half_h = boxes[:, 2] / 2, boxes[:, 3] / 2
    converted[:, 0] = boxes[:, 0] - half_w
    converted[:, 1] = boxes[:, 1] - half_h
    converted[:, 2] = boxes[:, 0] + half_w
    converted[:, 3] = boxes[:, 1] + half_h
    return converted


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    """Class-agnostic non-maximum suppression.

    Hand-rolled rather than cv.dnn.NMSBoxes: the dnn module is being reworked
    in OpenCV 5, and this is twelve lines that will not move under us.
    """
    x0, y0, x1, y1 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x1 - x0).clip(0) * (y1 - y0).clip(0)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        best = order[0]
        keep.append(int(best))
        if order.size == 1:
            break

        rest = order[1:]
        inter_w = (np.minimum(x1[best], x1[rest]) - np.maximum(x0[best], x0[rest])).clip(0)
        inter_h = (np.minimum(y1[best], y1[rest]) - np.maximum(y0[best], y0[rest])).clip(0)
        intersection = inter_w * inter_h
        union = areas[best] + areas[rest] - intersection
        overlap = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
        order = rest[overlap <= iou_threshold]
    return keep
