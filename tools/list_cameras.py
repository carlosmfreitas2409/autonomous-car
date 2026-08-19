"""Identify which camera index is the drive camera and which is the sign camera.

macOS reorders capture device indices between reboots and replugs, so
DRIVE_CAM_INDEX and SIGN_CAM_INDEX in src/main.py need checking whenever the
hardware changes.

    python3 tools/list_cameras.py
    python3 tools/list_cameras.py --preview     # windows labelled by index
"""

import argparse
import cv2 as cv


def probe(index: int):
    capture = cv.VideoCapture(index)
    if not capture.isOpened():
        capture.release()
        return None

    ok, frame = capture.read()
    info = {
        "index": index,
        "width": int(capture.get(cv.CAP_PROP_FRAME_WIDTH)),
        "height": int(capture.get(cv.CAP_PROP_FRAME_HEIGHT)),
        "fps": capture.get(cv.CAP_PROP_FPS),
        "frame": frame if ok else None,
    }
    capture.release()
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--max-index", type=int, default=6)
    parser.add_argument("--preview", action="store_true", help="show a labelled window per camera")
    args = parser.parse_args()

    found = []
    for index in range(args.max_index):
        info = probe(index)
        if info is None:
            continue
        found.append(info)
        print(f"[{info['index']}] {info['width']}x{info['height']} @ {info['fps']:.0f} fps")

    if not found:
        print("No cameras found.")
        return

    if args.preview:
        print("\nPoint each camera somewhere recognisable. Any key closes the windows.")
        for info in found:
            if info["frame"] is None:
                continue
            frame = info["frame"].copy()
            cv.putText(frame, f"index {info['index']}", (20, 60), cv.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
            cv.imshow(f"camera {info['index']}", frame)
        cv.waitKey(0)
        cv.destroyAllWindows()

    print("\nSet DRIVE_CAM_INDEX and SIGN_CAM_INDEX in src/main.py to match.")


if __name__ == "__main__":
    main()
