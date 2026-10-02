#!/usr/bin/env python3
"""Precision and recall on the real photos, and the latency that picks n or s.

The previous model reported mAP50 0.97 on a validation split its own compositor
generated, from its own cutouts, and found six of twenty-eight real photos. So
this never scores a model on synthetic data. It scores it on `assets/real`,
which is hand-labelled, and it reports:

  * an F1 sweep over the confidence threshold, so the runtime threshold is
    measured rather than guessed;
  * recall bucketed by apparent size, because a model that is excellent at
    120 px and blind at 500 px passes an averaged score and drives through the
    light;
  * inference latency on this machine, which is the number that decides
    whether the `s` variant's extra capacity is affordable.

    python3 tools/eval_signs.py --model models/signs_n.onnx
    python3 tools/eval_signs.py --model models/signs_s.onnx --dataset assets/real
"""

import argparse
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2 as cv
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.sign_detection import KNOWN_LABELS, SignDetector  # noqa: E402

IOU_MATCH = 0.5
SWEEP = np.round(np.arange(0.05, 0.91, 0.05), 2)
SIZE_BUCKETS = ((0, 40), (40, 90), (90, 180), (180, 320), (320, 10000))


def load_truth(label_path: Path, shape):
    if not label_path.exists():
        return None                      # not labelled at all - skip the image
    height, width = shape[:2]
    boxes = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        index = int(parts[0])
        cx, cy, w, h = (float(v) for v in parts[1:])
        boxes.append((KNOWN_LABELS[index],
                      ((cx - w / 2) * width, (cy - h / 2) * height,
                       (cx + w / 2) * width, (cy + h / 2) * height)))
    return boxes                         # may be empty: that is a negative


def iou(a, b) -> float:
    inter_w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = inter_w * inter_h
    if intersection <= 0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return intersection / (area_a + area_b - intersection)


def bucket_of(height: float) -> str:
    for low, high in SIZE_BUCKETS:
        if low <= height < high:
            return f"{low}-{high if high < 10000 else ''}px"
    return "?"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="models/signs.onnx")
    parser.add_argument("--dataset", default="assets/real")
    parser.add_argument("--confidence", type=float, default=0.05,
                        help="floor for the sweep; every detection above it is collected once")
    args = parser.parse_args()

    root = Path(args.dataset)
    images = sorted(p for p in (root / "images").rglob("*") if p.suffix.lower() in (".jpg", ".png"))
    if not images:
        print(f"no images under {root / 'images'}")
        return 2

    detector = SignDetector(args.model, confidence=args.confidence)
    if not detector.available:
        print(f"no usable model at {args.model}")
        return 2

    # Collect every detection once at the floor, then sweep offline.
    collected = []
    truths = []
    latencies = []
    labelled = 0
    for path in images:
        frame = cv.imread(str(path))
        truth = load_truth((root / "labels") / path.relative_to(root / "images").with_suffix(".txt"),
                           frame.shape)
        if truth is None:
            continue
        labelled += 1
        start = time.perf_counter()
        detections = detector.detect(frame)
        latencies.append((time.perf_counter() - start) * 1000)
        collected.append(detections)
        truths.append(truth)

    if not labelled:
        print(f"no labels under {root / 'labels'} - run tools/label_boxes.py first")
        return 2

    print(f"{labelled} labelled images, {sum(len(t) for t in truths)} objects\n")

    print("confidence sweep")
    print("  conf    TP    FP    FN   precision  recall      F1")
    best = (0.0, None)
    for threshold in SWEEP:
        tp, fp, fn = _score(collected, truths, threshold)
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
        print(f"  {threshold:4.2f} {tp:5d} {fp:5d} {fn:5d}     {precision:7.3f} {recall:7.3f} {f1:7.3f}")
        if f1 > best[0]:
            best = (f1, threshold)

    print(f"\nbest F1 {best[0]:.3f} at confidence {best[1]:.2f}")

    _per_class(collected, truths, best[1])
    _per_size(collected, truths, best[1])

    latency = np.array(latencies)
    print(f"\nlatency on this machine: median {np.median(latency):.0f} ms, "
          f"p90 {np.percentile(latency, 90):.0f} ms  ({1000 / max(1e-6, np.median(latency)):.1f} fps)")
    return 0


def _match(detections, truth, threshold):
    """Greedy highest-confidence-first matching at IoU 0.5, same class."""
    kept = sorted([d for d in detections if d.confidence >= threshold],
                  key=lambda d: -d.confidence)
    unmatched = list(truth)
    pairs, spurious = [], []
    for detection in kept:
        best, best_iou = None, 0.0
        for candidate in unmatched:
            if candidate[0] != detection.label:
                continue
            score = iou(detection.bbox, candidate[1])
            if score > best_iou:
                best, best_iou = candidate, score
        if best is not None and best_iou >= IOU_MATCH:
            unmatched.remove(best)
            pairs.append((detection, best))
        else:
            spurious.append(detection)
    return pairs, spurious, unmatched


def _score(collected, truths, threshold):
    tp = fp = fn = 0
    for detections, truth in zip(collected, truths):
        pairs, spurious, missed = _match(detections, truth, threshold)
        tp += len(pairs)
        fp += len(spurious)
        fn += len(missed)
    return tp, fp, fn


def _per_class(collected, truths, threshold) -> None:
    stats = defaultdict(lambda: [0, 0, 0])
    for detections, truth in zip(collected, truths):
        pairs, spurious, missed = _match(detections, truth, threshold)
        for _, matched in pairs:
            stats[matched[0]][0] += 1
        for detection in spurious:
            stats[detection.label][1] += 1
        for item in missed:
            stats[item[0]][2] += 1
    print(f"\nper class at {threshold:.2f}")
    print("  class             TP   FP   FN   recall")
    for label in KNOWN_LABELS:
        tp, fp, fn = stats[label]
        if tp + fp + fn == 0:
            print(f"  {label:<16s}  -    -    -    no instances in assets/real")
            continue
        recall = tp / (tp + fn) if tp + fn else 0.0
        print(f"  {label:<16s} {tp:4d} {fp:4d} {fn:4d}   {recall:6.2f}")


def _per_size(collected, truths, threshold) -> None:
    stats = defaultdict(lambda: [0, 0])
    for detections, truth in zip(collected, truths):
        pairs, _, missed = _match(detections, truth, threshold)
        for _, matched in pairs:
            stats[bucket_of(matched[1][3] - matched[1][1])][0] += 1
        for item in missed:
            stats[bucket_of(item[1][3] - item[1][1])][0] += 0
        for item in missed:
            stats[bucket_of(item[1][3] - item[1][1])][1] += 1
    print(f"\nrecall by apparent height at {threshold:.2f}")
    for low, high in SIZE_BUCKETS:
        key = f"{low}-{high if high < 10000 else ''}px"
        found, missed = stats[key]
        total = found + missed
        if not total:
            continue
        print(f"  {key:<12s} {found:3d}/{total:<3d}  {found / total:5.2f}")


if __name__ == "__main__":
    raise SystemExit(main())
