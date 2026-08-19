"""Measure the sign camera's focal length so distances come out in real metres.

Distance to a sign is recovered with the pinhole model:

    distance_m = real_height_m * focal_px / box_height_px

which needs focal_px. It can be derived from a field-of-view figure off a
datasheet, but lens specs are optimistic and the number matters — it scales
every braking distance. One measurement against a tape measure replaces the
guess.

Put an object of known height at a known distance, drag a box around it:

    python3 tools/calibrate_sign_camera.py --distance 2.0 --height 0.15
    python3 tools/calibrate_sign_camera.py --distance 2.0 --height 0.15 --image shot.jpg

Then paste the reported SIGN_CAM_HFOV_DEG into src/perception/tracking.py.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2 as cv

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.tracking import distance_from_height  # noqa: E402


def grab_frame(args):
    if args.image:
        frame = cv.imread(args.image)
        if frame is None:
            raise SystemExit(f"Could not read {args.image}")
        return frame

    capture = cv.VideoCapture(args.camera)
    if not capture.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}")

    capture.set(cv.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, args.height_px)
    print("Lining up. SPACE captures, q quits.")

    frame = None
    while True:
        ok, live = capture.read()
        if not ok:
            raise SystemExit("Camera stopped delivering frames")
        cv.imshow("calibration - SPACE to capture", live)
        key = cv.waitKey(1) & 0xFF
        if key == ord(" "):
            frame = live
            break
        if key == ord("q"):
            break

    capture.release()
    cv.destroyAllWindows()
    if frame is None:
        raise SystemExit("Cancelled")
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--distance", type=float, required=True, help="metres from the lens to the object")
    parser.add_argument("--height", type=float, required=True, help="real height of the object in metres")
    parser.add_argument("--camera", type=int, default=1, help="sign camera index")
    parser.add_argument("--image", help="calibrate from a saved photo instead of live capture")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height-px", type=int, default=720)
    args = parser.parse_args()

    frame = grab_frame(args)

    print("Drag a box around the object, then press ENTER.")
    x, y, w, box_height = cv.selectROI("select the object", frame, showCrosshair=False)
    cv.destroyAllWindows()
    if box_height <= 0:
        raise SystemExit("No box selected")

    frame_width = frame.shape[1]
    focal_px = box_height * args.distance / args.height
    hfov_deg = math.degrees(2 * math.atan(frame_width / (2 * focal_px)))

    print(f"\nBox height      {box_height} px in a {frame_width} px wide frame")
    print(f"Focal length    {focal_px:.1f} px")
    print(f"Field of view   {hfov_deg:.1f} degrees horizontal")
    print(f"\nCheck: that box implies {distance_from_height(args.height, focal_px, box_height):.2f} m "
          f"(measured {args.distance:.2f} m)")
    print("\nPaste into src/perception/tracking.py:")
    print(f"    SIGN_CAM_HFOV_DEG = {hfov_deg:.1f}")
    print("\nMeasure each sign once it is built and update REAL_HEIGHT_M in the same file.")


if __name__ == "__main__":
    main()
