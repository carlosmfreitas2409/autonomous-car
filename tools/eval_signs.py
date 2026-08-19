"""Score the exported model against real photos, and pick the threshold.

Two things make this worth having rather than trusting `yolo val`:

  It evaluates the deployed path. Detections come from src/perception's own
  SignDetector -- the same ONNX session, letterboxing, decode, NMS, and label
  aliasing that runs on the car. A model that scores well in ultralytics but
  badly here means the bug is in our code, and that is exactly what we want to
  find out before driving.

  It picks the confidence threshold from data. The previous attempt shipped
  conf=0.35 with nothing behind it. This sweeps thresholds and reports the F1
  at each, so config.sign_confidence becomes a measured number.

Point it at real, hand-labelled photos in YOLO format -- not the synthetic val
split, which flatters itself:

    dataset/real/images/*.jpg
    dataset/real/labels/*.txt
    dataset/real/data.yaml     (or pass --names stop,detour,traffic_light)

    python3 tools/eval_signs.py --dataset dataset/real
    python3 tools/eval_signs.py --dataset dataset/real --annotate /tmp/eval
"""

import argparse
import re
import sys
from pathlib import Path

import cv2 as cv

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.sign_detection import SignDetector  # noqa: E402
from perception.tracking import distance_from_height, focal_px_from_hfov  # noqa: E402

IOU_MATCH = 0.5
THRESHOLDS = [round(0.05 * i, 2) for i in range(1, 19)]

# Recall is reported per apparent size as well as overall, because "does it
# work" is really "out to what distance does it work". A class that is only
# separable close up shows here as recall collapsing in the small buckets,
# which is the signal to split it into detect-then-identify rather than asking
# the detector for a fine distinction it cannot make at range.
SIZE_BUCKETS = ((0, 20), (20, 35), (35, 60), (60, 120), (120, 10 ** 9))


def read_names(dataset: Path, override: str | None) -> list[str]:
    if override:
        return [n.strip() for n in override.split(",") if n.strip()]

    data_yaml = dataset / "data.yaml"
    if not data_yaml.exists():
        raise SystemExit(f"No {data_yaml} and no --names given")

    names: dict[int, str] = {}
    in_names = False
    for line in data_yaml.read_text().splitlines():
        if line.startswith("names:"):
            in_names = True
            continue
        if in_names:
            match = re.match(r"\s+(\d+):\s*(\S+)", line)
            if not match:
                break
            names[int(match.group(1))] = match.group(2)
    if not names:
        raise SystemExit(f"Could not read class names from {data_yaml}")
    return [names[i] for i in sorted(names)]


def read_labels(path: Path, names: list[str], width: int, height: int):
    if not path.exists():
        return []
    boxes = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        class_id, cx, cy, w, h = int(parts[0]), *map(float, parts[1:])
        boxes.append((
            names[class_id],
            (
                int((cx - w / 2) * width), int((cy - h / 2) * height),
                int((cx + w / 2) * width), int((cy + h / 2) * height),
            ),
        ))
    return boxes


def iou(a, b) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    inter_w = min(ax1, bx1) - max(ax0, bx0)
    inter_h = min(ay1, by1) - max(ay0, by0)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0
    intersection = inter_w * inter_h
    union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - intersection
    return intersection / union if union > 0 else 0.0


def score(samples, threshold: float, names: list[str]) -> dict:
    counts = {name: {"tp": 0, "fp": 0, "fn": 0} for name in names}

    for detections, truth in samples:
        predictions = sorted(
            (d for d in detections if d.confidence >= threshold),
            key=lambda d: d.confidence,
            reverse=True,
        )
        unmatched = list(truth)
        for prediction in predictions:
            best, best_overlap = None, IOU_MATCH
            for candidate in unmatched:
                if candidate[0] != prediction.label:
                    continue
                overlap = iou(prediction.bbox, candidate[1])
                if overlap >= best_overlap:
                    best, best_overlap = candidate, overlap
            if best is not None:
                unmatched.remove(best)
                counts.setdefault(prediction.label, {"tp": 0, "fp": 0, "fn": 0})["tp"] += 1
            else:
                counts.setdefault(prediction.label, {"tp": 0, "fp": 0, "fn": 0})["fp"] += 1
        for leftover in unmatched:
            counts.setdefault(leftover[0], {"tp": 0, "fp": 0, "fn": 0})["fn"] += 1

    return counts


def recall_by_size(samples, threshold: float, names: list[str]) -> dict:
    """Per class, how often a sign of a given pixel height was found at all."""
    tally = {name: {bucket: [0, 0] for bucket in SIZE_BUCKETS} for name in names}

    for detections, truth in samples:
        predictions = [d for d in detections if d.confidence >= threshold]
        for label, box in truth:
            if label not in tally:
                continue
            height = box[3] - box[1]
            bucket = next(b for b in SIZE_BUCKETS if b[0] <= height < b[1])
            tally[label][bucket][1] += 1
            if any(p.label == label and iou(p.bbox, box) >= IOU_MATCH for p in predictions):
                tally[label][bucket][0] += 1

    return tally


def report_by_size(samples, threshold: float, names: list[str], real_height_m: float | None) -> None:
    tally = recall_by_size(samples, threshold, names)
    frame_width = 1280

    print(f"\nRecall by apparent size, at confidence {threshold:.2f}")
    header = f"{'height':>12} " + "".join(f"{name[:11]:>13}" for name in names)
    if real_height_m:
        header += f"{'approx':>12}"
    print(header)
    print("-" * len(header))

    for bucket in SIZE_BUCKETS:
        low, high = bucket
        label = f"{low}-{high} px" if high < 10 ** 9 else f"{low}+ px"
        row = f"{label:>12} "
        for name in names:
            found, total = tally[name][bucket]
            row += f"{(f'{found / total:.2f} ({total})' if total else '-'):>13}"
        if real_height_m:
            # Midpoint of the bucket, turned back into a distance.
            mid = (low + min(high, 400)) / 2
            focal = focal_px_from_hfov(frame_width)
            distance = distance_from_height(real_height_m, focal, mid)
            row += f"{f'~{distance:.1f} m':>12}" if distance else f"{'':>12}"
        print(row)

    print("\nRecall collapsing in the small buckets for one class but not others "
          "means the detector cannot separate them at range. That is the case for "
          "detect-the-family-then-identify-up-close, not for more training.")


def rates(count: dict) -> tuple[float, float, float]:
    precision = count["tp"] / (count["tp"] + count["fp"]) if count["tp"] + count["fp"] else 0.0
    recall = count["tp"] / (count["tp"] + count["fn"]) if count["tp"] + count["fn"] else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return precision, recall, f1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--model", default="models/signs.onnx")
    parser.add_argument("--names", help="comma-separated class names, overriding data.yaml")
    parser.add_argument("--annotate", type=Path, help="write annotated images here")
    parser.add_argument("--real-height", type=float,
                        help="sign height in metres, to show the size buckets as distances")
    args = parser.parse_args()

    names = read_names(args.dataset, args.names)
    images = sorted((args.dataset / "images").glob("*.jpg")) + sorted((args.dataset / "images").glob("*.png"))
    if not images:
        raise SystemExit(f"No images in {args.dataset / 'images'}")

    # A permissive threshold here; the sweep below does the filtering.
    detector = SignDetector(args.model, confidence=0.01)
    if not detector.available:
        raise SystemExit(f"Could not load {args.model}. Run tools/train_signs.py first.")

    if args.annotate:
        args.annotate.mkdir(parents=True, exist_ok=True)

    samples = []
    for image_path in images:
        image = cv.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        detections = detector.detect(image)
        truth = read_labels(args.dataset / "labels" / f"{image_path.stem}.txt", names, width, height)
        samples.append((detections, truth))

        if args.annotate:
            annotated = image.copy()
            for label, box in truth:
                cv.rectangle(annotated, box[:2], box[2:], (0, 200, 0), 2)
            for detection in detections:
                if detection.confidence < 0.25:
                    continue
                cv.rectangle(annotated, detection.bbox[:2], detection.bbox[2:], (0, 120, 255), 2)
                cv.putText(annotated, f"{detection.label} {detection.confidence:.2f}",
                           (detection.bbox[0], max(14, detection.bbox[1] - 5)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 255), 1, cv.LINE_AA)
            cv.imwrite(str(args.annotate / image_path.name), annotated)

    print(f"{len(samples)} image(s), classes: {', '.join(names)}")
    print("green = ground truth, orange = prediction\n" if args.annotate else "")

    header = "conf  " + "".join(f"{name[:12]:>26}" for name in names) + "     mean F1"
    print(header)
    print("-" * len(header))

    best_threshold, best_f1 = None, -1.0
    for threshold in THRESHOLDS:
        counts = score(samples, threshold, names)
        cells, f1s = "", []
        for name in names:
            precision, recall, f1 = rates(counts[name])
            f1s.append(f1)
            cells += f"{f'P{precision:.2f} R{recall:.2f} F{f1:.2f}':>26}"
        mean_f1 = sum(f1s) / len(f1s) if f1s else 0.0
        print(f"{threshold:.2f}  {cells}     {mean_f1:.3f}")
        if mean_f1 > best_f1:
            best_threshold, best_f1 = threshold, mean_f1

    report_by_size(samples, best_threshold, names, args.real_height)

    print(f"\nBest mean F1 {best_f1:.3f} at confidence {best_threshold:.2f}")
    print(f"Set sign_confidence = {best_threshold:.2f} in src/config.py")
    print("\nRecall matters more than precision here: a missed stop sign is a run "
          "red light, a false one is a needless pause. Prefer a threshold whose "
          "recall is >= 0.9 even if precision suffers.")


if __name__ == "__main__":
    main()
