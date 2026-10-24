"""YOLO sign detection through onnxruntime.

Runtime is ONNX rather than ultralytics: ~30 MB against ~2.5 GB, no torch
import on startup, and the same artefact runs unchanged wherever this lands.
Training and export stay offline in tools/ on the machine with the GPU.

The detector is synchronous and owns no camera and no thread, so a test or an
offline eval can call it directly. If the model file or onnxruntime is missing
it reports itself unavailable and returns nothing, which keeps a fresh clone
runnable with no model trained.

Four classes, deliberately. The three delivery variants (pentagon B, hexagon A,
heptagon C) collapse into one `delivery_sign`, and the light's state is not a
class either - both are read from the crop by traffic_light.py and
delivery_variant.py. One extra polygon side at 30 px is not something a
detector head should be asked to bet a routing decision on.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import ast
import logging

import cv2 as cv
import numpy as np

logger = logging.getLogger(__name__)


# Square input the model is exported at. 640 is not negotiable: at 320 the
# traffic light was missed outright, which is the one thing that must not
# happen.
INPUT_SIZE = 640

# Grey letterbox padding, the value ultralytics trains against.
PAD_VALUE = 114

NMS_IOU = 0.45

# Everything downstream knows how to act on. Anything else the model emits is
# dropped here. These names match the directory names under
# assets/sign_cutouts/ and the class order written by tools/synthesize_dataset.py.
KNOWN_LABELS = ("delivery_sign", "detour", "stop", "traffic_light")

# A stock COCO model names two of these differently. Mapping them means the
# bootstrap weights and a fine-tuned model take the same code path, which is
# what lets the inspector run before anything has been trained.
_LABEL_ALIASES = {
    "stop sign": "stop",
    "traffic light": "traffic_light",
}

_PREFERRED_PROVIDERS = ("CoreMLExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider")


@dataclass
class Detection:
    label: str
    bbox: tuple[int, int, int, int]  # x0, y0, x1, y1 in source-frame pixels
    confidence: float

    @property
    def width(self) -> int:
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]

    def crop(self, frame: np.ndarray, margin: float = 0.0) -> np.ndarray:
        """The pixels this box covers, optionally with a margin around it.

        The traffic light reader needs margin: the colour it reads lives in the
        halo, which spills outside the housing.
        """
        height, width = frame.shape[:2]
        pad_x = int(round(self.width * margin))
        pad_y = int(round(self.height * margin))
        x0 = max(0, self.bbox[0] - pad_x)
        y0 = max(0, self.bbox[1] - pad_y)
        x1 = min(width, self.bbox[2] + pad_x)
        y1 = min(height, self.bbox[3] + pad_y)
        return frame[y0:y1, x0:x1]


class SignDetector:
    def __init__(self, model_path: str, confidence: float = 0.45):
        self._confidence = confidence
        self._session = None
        self._input_name: Optional[str] = None
        self._input_size = INPUT_SIZE
        self._labels: dict[int, str] = {}
        self._load(Path(model_path))

    @property
    def available(self) -> bool:
        return self._session is not None

    @property
    def confidence(self) -> float:
        return self._confidence

    def set_confidence(self, confidence: float) -> None:
        self._confidence = confidence

    def detect(self, frame: np.ndarray) -> list[Detection]:
        if self._session is None or frame is None or frame.size == 0:
            return []

        blob, scale, pad_x, pad_y = self._preprocess(white_balance(frame))
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
                logger.error("Could not load %s - sign detection disabled", model_path, exc_info=True)
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

        boxes = xywh_to_xyxy(predictions[keep, :4])
        confidences = confidences[keep]
        class_ids = class_ids[keep]

        # Undo the letterbox so boxes land back in source-frame pixels.
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / scale

        height, width = frame_shape
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, width - 1)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, height - 1)

        detections = []
        for index in non_max_suppression(boxes, confidences, NMS_IOU):
            label = self._labels.get(int(class_ids[index]))
            label = _LABEL_ALIASES.get(label, label)
            if label not in KNOWN_LABELS:
                continue
            x0, y0, x1, y1 = boxes[index].astype(int)
            if x1 <= x0 or y1 <= y0:
                continue
            detections.append(
                Detection(label, (int(x0), int(y0), int(x1), int(y1)), float(confidences[index]))
            )
        return detections


def white_balance(frame: np.ndarray) -> np.ndarray:
    """Grey-world white balance: scale each channel so the frame averages grey.

    The detector sees this; the crop readers never do - their colour rules were
    measured on the camera's own pixels. At the competition venue purple stage
    light through haze turns the red stop sign pink - face BGR (198, 135, 240)
    against the artwork's (39, 40, 217) - and round 6 found the stop in 14 of
    51 frames. Balanced, 33 of 51, detour 22 -> 25 of 26, delivery 36 -> 41 of
    60, with the earlier venue unchanged. Nothing to tune: the gains come from
    the frame itself, and the training set is balanced the same way so the
    model only ever learns from what it will be shown.
    """
    means = frame.reshape(-1, 3).mean(axis=0)
    gains = means.mean() / np.maximum(means, 1.0)
    return np.clip(frame.astype(np.float32) * gains, 0, 255).astype(np.uint8)


def letterbox(frame: np.ndarray, width: int, height: int, pad_value: int = PAD_VALUE):
    """Fit a frame into a canvas without distorting it.

    Returns the canvas plus the scale and offsets needed to map coordinates
    back and forth, so the model input and any debug view agree.
    """
    source_h, source_w = frame.shape[:2]
    scale = min(width / source_w, height / source_h)
    new_w, new_h = max(1, int(round(source_w * scale))), max(1, int(round(source_h * scale)))

    canvas = np.full((height, width, 3), pad_value, dtype=np.uint8)
    pad_x, pad_y = (width - new_w) // 2, (height - new_h) // 2
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv.resize(frame, (new_w, new_h))
    return canvas, scale, pad_x, pad_y


def xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    converted = np.empty_like(boxes)
    half_w, half_h = boxes[:, 2] / 2, boxes[:, 3] / 2
    converted[:, 0] = boxes[:, 0] - half_w
    converted[:, 1] = boxes[:, 1] - half_h
    converted[:, 2] = boxes[:, 0] + half_w
    converted[:, 3] = boxes[:, 1] + half_h
    return converted


def non_max_suppression(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    """Class-agnostic NMS.

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
