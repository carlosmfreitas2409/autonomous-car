#!/usr/bin/env python3


import argparse
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.sign_detection import KNOWN_LABELS, SignDetector  # noqa: E402

PLAUSIBLE_ASPECT = {
    "traffic_light": (1.6, 3.2),
    "stop": (0.6, 1.6),
    "detour": (0.6, 1.6),
    "delivery_sign": (0.6, 1.6),
}

SMALL_PX = 18


def load(dataset: Path):
    images_root, labels_root = dataset / "images", dataset / "labels"
    for image_path in sorted(images_root.rglob("*")):
        if image_path.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        label_path = labels_root / image_path.relative_to(images_root).with_suffix(".txt")
        yield image_path, label_path


def boxes_of(label_path: Path, shape):
    height, width = shape[:2]
    out = []
    if not label_path.exists():
        return out
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cx, cy, w, h = (float(v) for v in parts[1:])
        out.append((KNOWN_LABELS[int(parts[0])],
                    ((cx - w / 2) * width, (cy - h / 2) * height,
                     (cx + w / 2) * width, (cy + h / 2) * height)))
    return out


def iou(a, b) -> float:
    inter_w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = inter_w * inter_h
    if intersection <= 0:
        return 0.0
    union = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection)
    return intersection / union if union > 0 else 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="assets/real")
    parser.add_argument("--model", help="optional: also report where a model disagrees")
    parser.add_argument("--confidence", type=float, default=0.60,
                        help="a model detection this confident with no label is worth a look")
    parser.add_argument("--all", action="store_true",
                        help="draw every box, not just the suspicious ones - a triage list "
                             "cannot show you that nothing is wrong, only 65 tiles can")
    parser.add_argument("--sheet", help="default: .context/labels_<dataset>.png")
    args = parser.parse_args()

    dataset = Path(args.dataset)
    sheet = Path(args.sheet) if args.sheet else Path(f".context/labels_{dataset.name}.png")
    detector = None
    if args.model:
        detector = SignDetector(args.model, confidence=0.05)
        if not detector.available:
            print(f"no usable model at {args.model} - skipping the disagreement check\n")
            detector = None

    findings = []
    everything = []
    images = unlabelled = total_boxes = 0
    for image_path, label_path in load(dataset):
        frame = cv.imread(str(image_path))
        if frame is None:
            continue
        images += 1
        height, width = frame.shape[:2]
        where = "/".join(image_path.parts[-2:])

        if not label_path.exists():
            unlabelled += 1
            findings.append((3, where, None, "no label file at all - this image is silently skipped"))
            continue

        boxes = boxes_of(label_path, frame.shape)
        total_boxes += len(boxes)
        for label, box in boxes:
            everything.append((0, where, box, label))
        if not boxes:
            continue

        for index, (label, box) in enumerate(boxes):
            x0, y0, x1, y1 = box
            w, h = x1 - x0, y1 - y0
            if w <= 1 or h <= 1:
                findings.append((1, where, box, f"{label}: degenerate box {w:.0f}x{h:.0f}"))
                continue

            low, high = PLAUSIBLE_ASPECT.get(label, (0.2, 5.0))
            if not low <= h / w <= high:
                findings.append((1, where, box,
                                 f"{label}: aspect h/w {h / w:.2f}, outside {low}-{high}"))
            if min(w, h) < SMALL_PX:
                findings.append((2, where, box, f"{label}: only {w:.0f}x{h:.0f} px"))

            touching = [side for side, near in (("left", x0 <= 1), ("top", y0 <= 1),
                                                ("right", x1 >= width - 1),
                                                ("bottom", y1 >= height - 1)) if near]
            if touching:
                findings.append((2, where, box,
                                 f"{label}: touches the {' and '.join(touching)} edge, so the "
                                 f"object is cut off and the box is a guess"))

            for other_index, (other_label, other) in enumerate(boxes):
                if other_index <= index:
                    continue
                if iou(box, other) > 0.55:
                    findings.append((1, where, box,
                                     f"{label} and {other_label} overlap at IoU "
                                     f"{iou(box, other):.2f} - one may be a duplicate"))

        if detector is not None:
            found = detector.detect(frame)
            for detection in found:
                if detection.confidence < args.confidence:
                    continue
                if not any(detection.label == label and iou(detection.bbox, box) >= 0.4
                           for label, box in boxes):
                    findings.append((2, where, detection.bbox,
                                     f"model sees {detection.label} at {detection.confidence:.2f} "
                                     f"where nothing is labelled"))
            for label, box in boxes:
                if not any(d.label == label and iou(d.bbox, box) >= 0.4 for d in found):
                    findings.append((3, where, box,
                                     f"{label} labelled here, model sees nothing at all "
                                     f"(may be the model's fault, not the label's)"))

    print(f"{images} images, {total_boxes} boxes, {unlabelled} with no label file\n")
    if not findings:
        print("nothing surprising")
        return 0

    findings.sort(key=lambda f: f[0])
    severity = {1: "WRONG?", 2: "CHECK ", 3: "note  "}
    for level, where, _, message in findings:
        print(f"  {severity[level]} {where:34s} {message}")

    drawn = everything if args.all else [f for f in findings if f[2] is not None]
    if drawn:
        _sheet(dataset, drawn, sheet)
        what = f"all {len(drawn)} boxes" if args.all else f"{len(drawn)} suspects"
        print(f"\n{what} drawn in {sheet}")
    if not args.all:
        print("A triage list cannot show you that nothing is wrong. For that, --all.")
    print("\nFix any that are actually wrong with:")
    print(f"  python3 tools/label_boxes.py --images '{dataset}/images/**/*.jpg' \\")
    print(f"        --images-root {dataset}/images --labels-root {dataset}/labels")
    return 0


def _sheet(dataset: Path, findings, out: Path, tile: int = 240, columns: int = 6) -> None:
    rows = (len(findings) + columns - 1) // columns
    sheet = np.full((rows * (tile + 18), columns * tile, 3), 25, np.uint8)
    for index, (_, where, box, _) in enumerate(findings):
        frame = cv.imread(str(dataset / "images" / where))
        if frame is None:
            continue
        x0, y0, x1, y1 = (int(v) for v in box)
        cv.rectangle(frame, (x0, y0), (x1, y1), (60, 60, 255), 3)
        w, h = max(1, x1 - x0), max(1, y1 - y0)
        crop = frame[max(0, y0 - h):y1 + h, max(0, x0 - w):x1 + w]
        if crop.size == 0:
            continue
        scale = min(tile / crop.shape[1], tile / crop.shape[0])
        crop = cv.resize(crop, (max(1, int(crop.shape[1] * scale)),
                                max(1, int(crop.shape[0] * scale))))
        top, left = (index // columns) * (tile + 18), (index % columns) * tile
        sheet[top + 18:top + 18 + crop.shape[0], left:left + crop.shape[1]] = crop
        cv.putText(sheet, where.split("/")[-1][:26], (left + 2, top + 13),
                   cv.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv.imwrite(str(out), sheet)


if __name__ == "__main__":
    raise SystemExit(main())
