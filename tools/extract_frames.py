"""Pull frames out of a video.

Shooting video and extracting frames beats taking hundreds of stills: a
30-second walk around a sign at 30 fps is 900 frames of varied angle and
distance for half a minute of effort.

    python3 tools/extract_frames.py laps.mov --output dataset/backgrounds
    python3 tools/extract_frames.py signs.mov --output /tmp/signs --every 5 --max 200

Works on iPhone .mov straight off AirDrop.

Note on iPhone stills: photos come out as HEIC by default, which OpenCV cannot
read. Either set Settings > Camera > Formats > Most Compatible before shooting,
or convert afterwards:

    sips -s format jpeg *.HEIC --out converted/
"""

import argparse
from pathlib import Path

import cv2 as cv


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--every", type=int, default=10, help="keep one frame in N")
    parser.add_argument("--max", type=int, default=0, help="stop after this many frames (0 = no limit)")
    parser.add_argument("--width", type=int, default=1280, help="resize to this width (0 = leave alone)")
    args = parser.parse_args()

    if not args.video.exists():
        raise SystemExit(f"No such video: {args.video}")

    capture = cv.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"Could not open {args.video}. Converting to .mp4 usually fixes this.")

    args.output.mkdir(parents=True, exist_ok=True)
    stem = args.video.stem

    read = 0
    written = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        read += 1
        if read % args.every:
            continue

        if args.width and frame.shape[1] != args.width:
            height = round(frame.shape[0] * args.width / frame.shape[1])
            frame = cv.resize(frame, (args.width, height), interpolation=cv.INTER_AREA)

        cv.imwrite(str(args.output / f"{stem}_{written:05d}.jpg"), frame, [int(cv.IMWRITE_JPEG_QUALITY), 92])
        written += 1
        if args.max and written >= args.max:
            break

    capture.release()
    print(f"Read {read} frames, wrote {written} to {args.output}")


if __name__ == "__main__":
    main()
