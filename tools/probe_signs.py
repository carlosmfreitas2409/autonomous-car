#!/usr/bin/env python3
"""The retrain gate: pass or fail, non-zero exit on fail.

Three checks, all from things that have actually gone wrong here.

**Read (c) first.** Checks (a) and (b) paste cutouts through the same compositor
the model trained on, so they mark their own homework: the first model trained
from this pipeline passed (a) at 0.90-0.95 on lit cutouts while scoring 0.000
on the real photographs those cutouts were cut from. They catch a *scale*
regression, which is worth catching, and nothing else. Only (c) touches data
the training set has never seen.

**(a) Close range.** Every traffic-light cutout, pasted at 40 through 640 px,
must stay above 0.6 confidence *at every size*. The previous model was trained
with a 280 px cap and measured 0.97 at 260 px, 0.18 at 400 px and 0.00 at
500 px - it vanished exactly where the car has to stop, and an averaged metric
never showed it, because most of the validation set was small. The sign artwork
gets the same test up to 560 px: the signs had the same cap, at 260 px, and
this check only looked at the light, so it took the first real delivery photos
to find it.

**(c) Real photographs.** `traffic_light` recall on `assets/real`, which is
hand-labelled and never trained on. This is the only check here that can fail
for the right reason.

**(b) Bare emitters.** A lone lamp, a pair of lamps and an LED bar must not
reach the runtime threshold. The record has a bare red disc called `stop` at
0.86 and a green disc called `traffic_light` at 0.63. A vertical stack of three
lamps is deliberately not probed: that is the light's own silhouette.

    python3 tools/probe_signs.py --model models/signs.onnx --confidence 0.5
"""

import argparse
import random
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from perception.sign_detection import SignDetector  # noqa: E402
from synthesize_dataset import (  # noqa: E402
    background_frame, bloom, draw_distractors, load_backgrounds, render_onto)

SIZES = (40, 80, 140, 220, 320, 440, 560, 640)
SIGN_SIZES = (40, 80, 140, 220, 320, 440, 560)
SIGN_CLASSES = ("delivery_sign", "detour", "stop")
CLOSE_RANGE_FLOOR = 0.6

# What the model has to manage on photographs it has never seen. The plan's
# Gate 4 number.
REAL_RECALL_FLOOR = 0.9


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="models/signs.onnx")
    parser.add_argument("--confidence", type=float, default=0.5,
                        help="the runtime threshold the distractors must not reach")
    parser.add_argument("--cutouts", default="assets/sign_cutouts/traffic_light")
    parser.add_argument("--backgrounds", nargs="+", default=["dataset/backgrounds"])
    parser.add_argument("--dataset", default="assets/real",
                        help="hand-labelled real photographs for check (c)")
    parser.add_argument("--seed", type=int, default=3)
    args = parser.parse_args()

    detector = SignDetector(args.model, confidence=0.05)
    if not detector.available:
        print(f"FAIL: no usable model at {args.model}")
        return 2

    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    backgrounds = load_backgrounds(args.backgrounds)
    cutouts = sorted(Path(args.cutouts).glob("*.png"))
    if not cutouts or not backgrounds:
        print("FAIL: need traffic-light cutouts and backgrounds to probe with")
        return 2

    failures = []

    print(f"(a) every light cutout at every size, floor {CLOSE_RANGE_FLOOR}")
    worst = {}
    for path in cutouts:
        cutout = cv.imread(str(path), cv.IMREAD_UNCHANGED)
        if cutout is None or cutout.shape[2] != 4:
            continue
        for size in SIZES:
            frame = background_frame(rng.choice(backgrounds), rng)
            art = bloom(cutout, rng) if rng.random() < 0.5 else cutout
            box = render_onto(frame, art, size, rng,
                              position=(frame.shape[1] // 2 - size // 4,
                                        frame.shape[0] // 2 - size // 2),
                              box_alpha=cutout[:, :, 3])
            if box is None:
                continue
            found = max((d.confidence for d in detector.detect(frame)
                         if d.label == "traffic_light"), default=0.0)
            worst[size] = min(worst.get(size, 1.0), found)
            if found < CLOSE_RANGE_FLOOR:
                failures.append(f"{path.name} at {size}px -> {found:.2f}")
    for size in SIZES:
        if size in worst:
            mark = "ok " if worst[size] >= CLOSE_RANGE_FLOOR else "BAD"
            print(f"  {mark} {size:4d} px   worst confidence {worst[size]:.2f}")

    print(f"\n    every sign artwork at every size, floor {CLOSE_RANGE_FLOOR}")
    signs_root = Path(args.cutouts).parent
    for label in SIGN_CLASSES:
        worst = {}
        for path in sorted((signs_root / label).glob("*.png")):
            cutout = cv.imread(str(path), cv.IMREAD_UNCHANGED)
            if cutout is None or cutout.shape[2] != 4:
                continue
            for size in SIGN_SIZES:
                frame = background_frame(rng.choice(backgrounds), rng)
                box = render_onto(frame, cutout, size, rng,
                                  position=(frame.shape[1] // 2 - size // 2,
                                            frame.shape[0] // 2 - size // 2))
                if box is None:
                    continue
                found = max((d.confidence for d in detector.detect(frame)
                             if d.label == label), default=0.0)
                worst[size] = min(worst.get(size, 1.0), found)
                if found < CLOSE_RANGE_FLOOR:
                    failures.append(f"{label}/{path.name} at {size}px -> {found:.2f}")
        if worst:
            row = "  ".join(f"{size}:{worst[size]:.2f}" for size in SIGN_SIZES if size in worst)
            mark = "ok " if min(worst.values()) >= CLOSE_RANGE_FLOOR else "BAD"
            print(f"  {mark} {label:14s} {row}")

    print(f"\n(b) bare emitters below the runtime threshold {args.confidence}")
    loudest = 0.0
    for trial in range(40):
        frame = background_frame(rng.choice(backgrounds), rng)
        draw_distractors(frame, rng)
        for detection in detector.detect(frame):
            if detection.confidence >= args.confidence:
                loudest = max(loudest, detection.confidence)
                failures.append(f"distractor frame {trial} -> {detection.label} "
                                f"{detection.confidence:.2f}")
    print(f"  {'BAD' if loudest >= args.confidence else 'ok '} loudest distractor "
          f"{loudest:.2f}")

    print(f"\n(c) real photographs, recall floor {REAL_RECALL_FLOOR}")
    recall, found, total, lit_recall = _real_recall(detector, Path(args.dataset), args.confidence)
    if total:
        mark = "ok " if recall >= REAL_RECALL_FLOOR else "BAD"
        print(f"  {mark} {found}/{total} found at confidence {args.confidence}   recall {recall:.2f}")
        print(f"      of those with a lamp lit: {lit_recall:.2f}")
        if recall < REAL_RECALL_FLOOR:
            failures.append(f"real-photo recall {recall:.2f} < {REAL_RECALL_FLOOR}")
    else:
        print(f"  ?? no labelled photographs under {args.dataset} - (c) skipped, so this")
        print("     run only tells you about composited data")

    print()
    if failures:
        print(f"FAIL - {len(failures)} problems")
        for line in failures[:20]:
            print(f"  {line}")
        if len(failures) > 20:
            print(f"  ... and {len(failures) - 20} more")
        return 1
    print("PASS")
    return 0


def _real_recall(detector, root: Path, threshold: float):
    """How many hand-labelled traffic lights the model actually finds."""
    images = sorted(p for p in (root / "images").rglob("*.jpg"))
    found = total = lit_found = lit_total = 0
    for path in images:
        label = (root / "labels") / path.relative_to(root / "images").with_suffix(".txt")
        if not label.exists():
            continue
        frame = cv.imread(str(path))
        height, width = frame.shape[:2]
        for line in label.read_text().splitlines():
            parts = line.split()
            if len(parts) != 5 or parts[0] != "3":
                continue
            cx, cy, w, h = (float(v) for v in parts[1:])
            truth = ((cx - w / 2) * width, (cy - h / 2) * height,
                     (cx + w / 2) * width, (cy + h / 2) * height)
            hit = any(_iou(d.bbox, truth) >= 0.5
                      for d in detector.detect(frame)
                      if d.label == "traffic_light" and d.confidence >= threshold)
            total += 1
            found += hit
            if not path.stem.startswith("off"):
                lit_total += 1
                lit_found += hit
    return (found / total if total else 0.0, found, total,
            lit_found / lit_total if lit_total else 0.0)


def _iou(a, b) -> float:
    inter_w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = inter_w * inter_h
    if intersection <= 0:
        return 0.0
    union = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection)
    return intersection / union if union > 0 else 0.0


if __name__ == "__main__":
    raise SystemExit(main())
