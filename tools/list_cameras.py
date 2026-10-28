#!/usr/bin/env python3


import argparse

import cv2 as cv


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-index", type=int, default=6)
    parser.add_argument("--show", action="store_true", help="display one frame from each")
    args = parser.parse_args()

    found = 0
    for index in range(args.max_index):
        capture = cv.VideoCapture(index)
        if not capture.isOpened():
            capture.release()
            continue
        ok, frame = capture.read()
        width = int(capture.get(cv.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv.CAP_PROP_FRAME_HEIGHT))
        fps = capture.get(cv.CAP_PROP_FPS)
        print(f"  {index}: {width}x{height} @ {fps:.0f} fps  {'reads frames' if ok else 'OPENS BUT READS NOTHING'}")
        found += 1
        if args.show and ok:
            cv.imshow(f"camera {index} - any key to continue", frame)
            cv.waitKey(0)
            cv.destroyAllWindows()
        capture.release()

    if not found:
        print("  no cameras found")
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
