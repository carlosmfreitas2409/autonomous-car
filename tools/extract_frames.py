#!/usr/bin/env python3
"""Turn a clip into a folder of frames.

Thirty seconds of walking is about nine hundred varied frames; ten stills is
ten. Every shoot in this phase is video for that reason, and this is what turns
one into training backgrounds or into images to label.

    python3 tools/extract_frames.py clips/light_approach.mp4 --out dataset/backgrounds --every 6
    python3 tools/extract_frames.py clips/*.mov --out dataset/real/images/detour --every 12 --limit 120
"""

import argparse
from pathlib import Path

import cv2 as cv


def extract(clip: Path, out: Path, every: int, limit: int, prefix: str) -> int:
    capture = cv.VideoCapture(str(clip))
    if not capture.isOpened():
        print(f"could not open {clip}")
        return 0
    out.mkdir(parents=True, exist_ok=True)
    index = written = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % every == 0:
            name = f"{prefix}{clip.stem}_{index:06d}.jpg"
            cv.imwrite(str(out / name), frame)
            written += 1
            if limit and written >= limit:
                break
        index += 1
    capture.release()
    print(f"{clip.name}: {written} frames of {index} -> {out}")
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("clips", nargs="+")
    parser.add_argument("--out", required=True)
    parser.add_argument("--every", type=int, default=6,
                        help="keep one frame in this many; consecutive frames are near-duplicates")
    parser.add_argument("--limit", type=int, default=0, help="per clip, 0 for all")
    parser.add_argument("--prefix", default="")
    args = parser.parse_args()

    total = sum(extract(Path(c), Path(args.out), args.every, args.limit, args.prefix)
                for c in args.clips)
    print(f"\n{total} frames written")
    return 0 if total else 1


if __name__ == "__main__":
    raise SystemExit(main())
