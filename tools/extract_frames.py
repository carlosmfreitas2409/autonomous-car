#!/usr/bin/env python3
"""Turn a clip, or a live camera, into a folder of frames.

Thirty seconds of walking is about nine hundred varied frames; ten stills is
ten. Every shoot in this phase is video for that reason, and this is what turns
one into training backgrounds or into images to label.

    python3 tools/extract_frames.py clips/light_approach.mp4 --out dataset/backgrounds --every 6
    python3 tools/extract_frames.py clips/*.mov --out assets/real/images/detour --every 12 --limit 120
    python3 tools/extract_frames.py --record 1 --out dataset/backgrounds --every 6

A recording uses the sign camera's settings (1280x720, MJPG, autofocus and
auto white balance off) and shows a preview. q or ESC stops it. A later run
keeps the frames already in --out and continues the numbering.
"""

import argparse
from pathlib import Path

import cv2 as cv


def extract(clip: Path, out: Path, every: int, limit: int, prefix: str) -> int:
    capture = cv.VideoCapture(str(clip))
    if not capture.isOpened():
        print(f"could not open {clip}")
        return 0
    written, seen = _take(capture, out, every, limit, prefix, clip.stem, None)
    capture.release()
    print(f"{clip.name}: {written} frames of {seen} -> {out}")
    return written


def record(camera: int, out: Path, every: int, limit: int, prefix: str) -> int:
    capture = cv.VideoCapture(camera)
    if not capture.isOpened():
        print(f"could not open camera {camera}")
        capture.release()
        return 0
    _configure_camera(capture)
    window = f"camera {camera}  —  q to stop"

    def preview(frame, written: int) -> bool:
        shown = frame.copy()
        cv.putText(shown, f"{written} saved   q to stop", (16, 32),
                   cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv.imshow(window, shown)
        return (cv.waitKey(1) & 0xFF) in (ord("q"), 27)

    try:
        written, seen = _take(capture, out, every, limit, prefix, f"cam{camera}", preview)
    finally:
        capture.release()
        cv.destroyAllWindows()
    if seen == 0:
        print(f"camera {camera} opened but reads nothing")
    else:
        print(f"camera {camera}: {written} frames of {seen} -> {out}")
    return written


def _take(capture, out: Path, every: int, limit: int, prefix: str, stem: str, preview):
    """Keep one frame in `every`, until the source ends, the limit, or preview says stop."""
    out.mkdir(parents=True, exist_ok=True)
    base = _next_index(out, prefix, stem, every)
    if base:
        print(f"existing frames kept; writing from {prefix}{stem}_{base:06d}.jpg")
    seen = written = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if seen % every == 0:
            cv.imwrite(str(out / f"{prefix}{stem}_{base + seen:06d}.jpg"), frame)
            written += 1
        seen += 1
        if preview is not None and preview(frame, written):
            break
        if limit and written >= limit:
            break
    return written, seen


def _next_index(out: Path, prefix: str, stem: str, every: int) -> int:
    """First free frame number after whatever this stem already wrote."""
    highest = -1
    needle = f"{prefix}{stem}_"
    for path in out.glob(f"{needle}*.jpg"):
        suffix = path.stem[len(needle):]
        if suffix.isdigit():
            highest = max(highest, int(suffix))
    if highest < 0:
        return 0
    return highest + every


def _configure_camera(capture) -> None:
    # Same settings as the sign camera, so a shoot matches what the car sees.
    settings = (
        ("MJPG", cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG")),
        ("1280 wide", cv.CAP_PROP_FRAME_WIDTH, 1280),
        ("720 high", cv.CAP_PROP_FRAME_HEIGHT, 720),
        ("buffer 1", cv.CAP_PROP_BUFFERSIZE, 1),
        ("autofocus off", cv.CAP_PROP_AUTOFOCUS, 0),
        ("auto white balance off", cv.CAP_PROP_AUTO_WB, 0),
    )
    ignored = [name for name, prop, value in settings if not capture.set(prop, value)]
    if ignored:
        print("camera ignored: " + ", ".join(ignored))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("clips", nargs="*", help="video files; omit when using --record")
    parser.add_argument("--record", type=int, metavar="INDEX",
                        help="record from this camera index instead of a file")
    parser.add_argument("--out", required=True)
    parser.add_argument("--every", type=int, default=6,
                        help="keep one frame in this many; consecutive frames are near-duplicates")
    parser.add_argument("--limit", type=int, default=0, help="per clip, 0 for all")
    parser.add_argument("--prefix", default="")
    args = parser.parse_args()

    if (args.record is not None) == bool(args.clips):
        parser.error("give clip files, or --record INDEX")

    if args.record is not None:
        total = record(args.record, Path(args.out), args.every, args.limit, args.prefix)
    else:
        total = sum(extract(Path(c), Path(args.out), args.every, args.limit, args.prefix)
                    for c in args.clips)
    print(f"\n{total} frames written")
    return 0 if total else 1


if __name__ == "__main__":
    raise SystemExit(main())
