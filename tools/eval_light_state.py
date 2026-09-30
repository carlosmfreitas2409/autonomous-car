#!/usr/bin/env python3
"""Grade the traffic-light state reader on the real venue photos.

Ground truth is free: every photo says what was lit in its own filename
(`red_l20.jpg`), and the one folder that came off the camera with timestamps
carries a `states.txt` beside it. An `off_*` photo is correct only when the
reader answers UNKNOWN - an unlit housing that reads RED is the failure that
mattered most in the previous attempt, and it has to cost a point here.

Box sourcing is detector first, hand label as fallback, and every row records
which one it used. Mixing them silently would hide a detector regression behind
a good reader score, which is exactly the trap this phase exists to avoid.

    python3 tools/eval_light_state.py                      # detector where it can
    python3 tools/eval_light_state.py --confidence 1.1     # hand boxes only
    python3 tools/eval_light_state.py --annotate out/      # a panel per failure
"""

import argparse
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2 as cv
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.sign_detection import SignDetector  # noqa: E402
from perception.traffic_light import CROP_MARGIN, read_light_state  # noqa: E402

STATES = ("RED", "YELLOW", "GREEN", "UNKNOWN")
COLUMNS = STATES + ("NO-BOX",)


def ground_truth(image_path: Path) -> str:
    """What was lit, from the filename or from the folder's states.txt."""
    sidecar = image_path.parent / "states.txt"
    if sidecar.exists():
        for line in sidecar.read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            name, _, state = line.partition(" ")
            if name == image_path.name:
                return "UNKNOWN" if state.strip() == "off" else state.strip().upper()
    head = image_path.stem.split("_")[0].lower()
    if head == "off":
        return "UNKNOWN"
    return head.upper() if head.upper() in STATES else "?"


def hand_box(image_path: Path, images_root: Path, labels_root: Path, shape):
    """The hand-drawn box for this image, in pixels, if there is one."""
    label = labels_root / image_path.relative_to(images_root).with_suffix(".txt")
    if not label.exists():
        return None
    height, width = shape
    for line in label.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cx, cy, w, h = (float(v) for v in parts[1:])
        return (int((cx - w / 2) * width), int((cy - h / 2) * height),
                int((cx + w / 2) * width), int((cy + h / 2) * height))
    return None


def crop_with_margin(frame, box, margin=CROP_MARGIN):
    x0, y0, x1, y1 = box
    height, width = frame.shape[:2]
    pad_x, pad_y = int((x1 - x0) * margin), int((y1 - y0) * margin)
    return frame[max(0, y0 - pad_y):min(height, y1 + pad_y),
                 max(0, x0 - pad_x):min(width, x1 + pad_x)]


def yaw_of(name: str) -> str:
    for tag in ("l40", "l20", "r20", "r40"):
        if tag in name:
            return tag
    return "head"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default="assets/real")
    parser.add_argument("--model", default="models/signs.onnx")
    parser.add_argument("--confidence", type=float, default=0.5,
                        help="detector threshold; above 1.0 forces hand boxes")
    parser.add_argument("--annotate", metavar="DIR",
                        help="write a crop|core|halo panel for every failure")
    args = parser.parse_args()

    root = Path(args.dataset)
    images_root, labels_root = root / "images", root / "labels"
    images = sorted(p for p in images_root.rglob("*.jpg"))
    if not images:
        print(f"no images under {images_root}")
        return 2

    detector = SignDetector(args.model, confidence=min(args.confidence, 0.99))
    use_detector = args.confidence <= 1.0 and detector.available
    if args.confidence > 1.0:
        print("detector disabled: grading the reader on hand boxes alone\n")
    elif not detector.available:
        print(f"no usable model at {args.model}: falling back to hand boxes\n")

    annotate_dir = Path(args.annotate) if args.annotate else None
    if annotate_dir:
        annotate_dir.mkdir(parents=True, exist_ok=True)

    matrix = defaultdict(Counter)
    by_bucket = defaultdict(lambda: [0, 0])
    by_yaw = defaultdict(lambda: [0, 0])
    box_source = Counter()
    detector_hits = defaultdict(lambda: [0, 0])
    failures = []

    for path in images:
        truth = ground_truth(path)
        if truth == "?":
            continue
        frame = cv.imread(str(path))
        bucket = path.parent.name

        box, source = None, "none"
        if use_detector:
            lights = [d for d in detector.detect(frame) if d.label == "traffic_light"]
            if lights:
                box = max(lights, key=lambda d: d.confidence).bbox
                source = "detector"
        detector_hits[bucket][1] += 1
        detector_hits[bucket][0] += source == "detector"
        if box is None:
            box = hand_box(path, images_root, labels_root, frame.shape[:2])
            source = "hand" if box else "none"
        box_source[source] += 1

        if box is None:
            predicted, reading = "NO-BOX", None
        else:
            reading = read_light_state(crop_with_margin(frame, box))
            predicted = reading.state.value

        matrix[truth][predicted] += 1
        correct = predicted == truth
        for table, key in ((by_bucket, bucket), (by_yaw, yaw_of(path.stem))):
            table[key][1] += 1
            table[key][0] += correct

        if not correct:
            failures.append((path, truth, predicted, source, box, reading))
            if annotate_dir is not None and reading is not None:
                _write_panel(annotate_dir, path, reading)

    _report(matrix, by_bucket, by_yaw, box_source, detector_hits, failures, len(images))
    # A report, not a gate: it exits 0 whenever it managed to measure something.
    # tools/probe_signs.py is the one that fails a build.
    return 0


def _write_panel(directory: Path, path: Path, reading) -> None:
    debug = reading.debug
    if not debug:
        return
    work = debug["work"]
    tiles = [work,
             cv.cvtColor(debug["core_mask"], cv.COLOR_GRAY2BGR),
             cv.cvtColor(debug["halo_mask"], cv.COLOR_GRAY2BGR)]
    height = max(t.shape[0] for t in tiles)
    scaled = []
    for tile in tiles:
        factor = height / tile.shape[0]
        scaled.append(cv.resize(tile, (int(tile.shape[1] * factor), height)))
    name = "_".join(path.parts[-2:]).replace(".jpg", ".png")
    cv.imwrite(str(directory / name), np.hstack(scaled))


def _report(matrix, by_bucket, by_yaw, box_source, detector_hits, failures, image_count) -> None:
    total = sum(sum(row.values()) for row in matrix.values())
    correct = sum(matrix[state][state] for state in matrix)
    print(f"{correct}/{total} correct over {image_count} photos "
          f"({dict(box_source)})\n")

    print("confusion  (rows: what was lit, columns: what was read)")
    print("           " + "".join(f"{c:>9s}" for c in COLUMNS))
    for truth in STATES:
        row = matrix.get(truth)
        if not row:
            continue
        cells = "".join(f"{row.get(c, 0):9d}" for c in COLUMNS)
        print(f"{truth:>10s} {cells}")

    lit_total = sum(sum(matrix[s].values()) for s in ("RED", "YELLOW", "GREEN"))
    lit_correct = sum(matrix[s][s] for s in ("RED", "YELLOW", "GREEN"))
    off_total = sum(matrix["UNKNOWN"].values())
    off_correct = matrix["UNKNOWN"]["UNKNOWN"]
    swaps = matrix["RED"]["GREEN"] + matrix["GREEN"]["RED"]
    print(f"\nlit photos read correctly   {lit_correct}/{lit_total}"
          f"{'' if not lit_total else f'  ({lit_correct / lit_total:.2f})'}")
    print(f"unlit photos read UNKNOWN   {off_correct}/{off_total}")
    print(f"red read as green, or back  {swaps}")

    print("\nby distance" + " " * 10 + "by yaw")
    buckets = sorted(by_bucket), sorted(by_yaw)
    for index in range(max(len(buckets[0]), len(buckets[1]))):
        left = right = ""
        if index < len(buckets[0]):
            key = buckets[0][index]
            hit, seen = by_bucket[key]
            detected, of = detector_hits[key]
            left = f"  {key:<6s} {hit:2d}/{seen:<2d}  detector {detected}/{of}"
        if index < len(buckets[1]):
            key = buckets[1][index]
            hit, seen = by_yaw[key]
            right = f"  {key:<5s} {hit:2d}/{seen:<2d}"
        print(f"{left:<36s}{right}")

    if not failures:
        print("\nno failures")
        return
    print(f"\n{len(failures)} failures")
    for path, truth, predicted, source, box, reading in failures:
        where = "/".join(path.parts[-2:])
        shape = "" if box is None else f"box={box[2] - box[0]}x{box[3] - box[1]}"
        cells = "-"
        if reading is not None and reading.cells is not None:
            cells = f"{reading.cells.found}/3 r={reading.cells.radius:.0f}"
        detail = "" if reading is None else (
            f" conf={reading.confidence:.2f} cells={cells}"
            f" pos={reading.position_evidence:.2f} col={reading.colour_evidence:.2f}"
            f" emit={list(reading.emission)} [{reading.reason}]")
        print(f"BAD  {where:<30s} gt={truth:<8s} pred={predicted:<8s} {shape} src={source}{detail}")


if __name__ == "__main__":
    raise SystemExit(main())
