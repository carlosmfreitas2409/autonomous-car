#!/usr/bin/env python3
"""The retrain gate: pass or fail, non-zero exit on fail.

Two checks, both from things that have actually gone wrong here.

**(a) Close range.** Every traffic-light cutout, pasted at 40 through 640 px,
must stay above 0.6 confidence *at every size*. The previous model was trained
with a 280 px cap and measured 0.97 at 260 px, 0.18 at 400 px and 0.00 at
500 px - it vanished exactly where the car has to stop, and an averaged metric
never showed it, because most of the validation set was small.

**(b) Bare emitters.** A lone lamp, a pair of lamps and an LED bar must not
reach the runtime threshold. The record has a bare red disc called `stop` at
0.86 and a green disc called `traffic_light` at 0.63. A vertical stack of three
lamps is deliberately not probed: that is the light's own silhouette.

    python3 tools/probe_signs.py --model models/signs.onnx --confidence 0.35
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
CLOSE_RANGE_FLOOR = 0.6


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="models/signs.onnx")
    parser.add_argument("--confidence", type=float, default=0.35,
                        help="the runtime threshold the distractors must not reach")
    parser.add_argument("--cutouts", default="assets/sign_cutouts/traffic_light")
    parser.add_argument("--backgrounds", nargs="+", default=["dataset/backgrounds"])
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


if __name__ == "__main__":
    raise SystemExit(main())
