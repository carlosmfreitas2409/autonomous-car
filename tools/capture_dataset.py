"""Record sign-camera frames of the track.

Worth doing before the signs exist. Background footage of the real track under
the real lighting is the scarce ingredient in the training set -- the signs
themselves get composited in later by synthesize_dataset.py, straight from the
artwork, with no hand labelling.

    python3 tools/capture_dataset.py                    # SPACE to grab
    python3 tools/capture_dataset.py --auto --every 8   # grab while driving

Drive a few laps in varied light. A few hundred frames is plenty.
"""

import argparse
import time
from pathlib import Path

import cv2 as cv


DEFAULT_OUTPUT = Path("dataset/backgrounds")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=1, help="sign camera index")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--auto", action="store_true", help="save continuously")
    parser.add_argument("--every", type=int, default=8, help="save one frame in N when auto")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    existing = len(list(args.output.glob("*.jpg")))

    capture = cv.VideoCapture(args.camera)
    if not capture.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}")
    capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv.CAP_PROP_BUFFERSIZE, 1)

    auto = args.auto
    saved = 0
    seen = 0
    print(f"{existing} frames already in {args.output}")
    print("SPACE saves one frame, A toggles auto, Q quits.")

    while True:
        ok, frame = capture.read()
        if not ok:
            print("Camera stopped delivering frames")
            break
        seen += 1

        should_save = auto and seen % args.every == 0
        key = cv.waitKey(1) & 0xFF
        if key == ord(" "):
            should_save = True
        elif key == ord("a"):
            auto = not auto
            print(f"auto {'on' if auto else 'off'}")
        elif key == ord("q"):
            break

        if should_save:
            name = args.output / f"{int(time.time() * 1000)}.jpg"
            cv.imwrite(str(name), frame)
            saved += 1

        overlay = frame.copy()
        status = f"saved {saved} (total {existing + saved})" + ("  [AUTO]" if auto else "")
        cv.putText(overlay, status, (16, 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv.imshow("capture - SPACE save, A auto, Q quit", overlay)

    capture.release()
    cv.destroyAllWindows()
    print(f"Saved {saved} frames to {args.output}")


if __name__ == "__main__":
    main()
