#!/usr/bin/env python3
"""Draw boxes on images and write YOLO labels.

About 250 images, almost all of them with a single object in frame. The round
trip through Roboflow or CVAT - upload, wait, configure a project, export, work
out which zip layout it chose this time - costs more than these hundred and
fifty lines. `--prefill` turns the job into accept-or-fix.

Keys
    1-4         pick the class
    drag        add a box in the current class
    u           undo the last box
    c           clear this image
    n / p       next / previous image
    s           save now (saving also happens on every move)
    q / ESC     quit

    python3 tools/label_boxes.py --images 'assets/real/images/**/*.jpg'
    python3 tools/label_boxes.py --images 'assets/real/images/**/*.jpg' --prefill models/signs.onnx
"""

import argparse
import glob
import sys
from pathlib import Path

import cv2 as cv

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.sign_detection import KNOWN_LABELS, SignDetector  # noqa: E402

RULE = """
  The box is the SIGN FACE or the LAMP HOUSING, and nothing else. Never the post.

  Phase-2 distance is real_height / pixel_height, which is only constant if the
  box always bounds the same physical object, and the light's state reader finds
  the lamp cells inside the box, which only works when the box is the housing.
"""

COLOURS = ((90, 220, 255), (0, 200, 255), (60, 60, 255), (120, 255, 120))
WINDOW = "label_boxes"
MAX_SIDE = 1100


def read_labels(path: Path, shape):
    if not path.exists():
        return []
    height, width = shape[:2]
    boxes = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        index = int(parts[0])
        cx, cy, w, h = (float(v) for v in parts[1:])
        boxes.append((index,
                      int((cx - w / 2) * width), int((cy - h / 2) * height),
                      int((cx + w / 2) * width), int((cy + h / 2) * height)))
    return boxes


def write_labels(path: Path, boxes, shape) -> None:
    height, width = shape[:2]
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for index, x0, y0, x1, y1 in boxes:
        cx, cy = (x0 + x1) / 2 / width, (y0 + y1) / 2 / height
        lines.append(f"{index} {cx:.6f} {cy:.6f} {(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}")
    # Written even when empty: an image with nothing in it is a negative, and
    # both the eval and ultralytics only count it as one if the file exists.
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def label_path(image: Path, images_root: Path, labels_root: Path) -> Path:
    try:
        relative = image.relative_to(images_root)
    except ValueError:
        relative = Path(image.name)
    return labels_root / relative.with_suffix(".txt")


def draw(frame, boxes, current, name, index, total, scale):
    canvas = cv.resize(frame, None, fx=scale, fy=scale) if scale != 1.0 else frame.copy()
    for class_index, x0, y0, x1, y1 in boxes:
        colour = COLOURS[class_index % len(COLOURS)]
        cv.rectangle(canvas, (int(x0 * scale), int(y0 * scale)),
                     (int(x1 * scale), int(y1 * scale)), colour, 2)
        cv.putText(canvas, KNOWN_LABELS[class_index], (int(x0 * scale), max(14, int(y0 * scale) - 6)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
    header = f"[{index + 1}/{total}] {name}   class {current + 1}: {KNOWN_LABELS[current]}   {len(boxes)} box(es)"
    cv.rectangle(canvas, (0, 0), (canvas.shape[1], 24), (0, 0, 0), -1)
    cv.putText(canvas, header, (6, 17), cv.FONT_HERSHEY_SIMPLEX, 0.5,
               COLOURS[current % len(COLOURS)], 1)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True, help="glob, e.g. 'assets/real/images/**/*.jpg'")
    parser.add_argument("--images-root", default="assets/real/images")
    parser.add_argument("--labels-root", default="assets/real/labels")
    parser.add_argument("--prefill", metavar="MODEL",
                        help="seed each unlabelled image with this model's boxes")
    parser.add_argument("--confidence", type=float, default=0.35)
    args = parser.parse_args()

    print(RULE)

    images = [Path(p) for p in sorted(glob.glob(args.images, recursive=True))]
    if not images:
        print(f"no images matched {args.images}")
        return 2
    images_root, labels_root = Path(args.images_root), Path(args.labels_root)

    detector = None
    if args.prefill:
        detector = SignDetector(args.prefill, confidence=args.confidence)
        if not detector.available:
            print(f"prefill model {args.prefill} unavailable - labelling from scratch")
            detector = None

    cv.namedWindow(WINDOW, cv.WINDOW_AUTOSIZE)
    state = {"drag": None, "boxes": [], "current": 0, "scale": 1.0}

    def on_mouse(event, x, y, flags, _):
        px, py = int(x / state["scale"]), int(y / state["scale"])
        if event == cv.EVENT_LBUTTONDOWN:
            state["drag"] = (px, py)
        elif event == cv.EVENT_LBUTTONUP and state["drag"]:
            x0, y0 = state["drag"]
            state["drag"] = None
            if abs(px - x0) > 4 and abs(py - y0) > 4:
                state["boxes"].append((state["current"], min(x0, px), min(y0, py),
                                       max(x0, px), max(y0, py)))

    cv.setMouseCallback(WINDOW, on_mouse)

    index = 0
    while 0 <= index < len(images):
        image_path = images[index]
        frame = cv.imread(str(image_path))
        if frame is None:
            index += 1
            continue
        target = label_path(image_path, images_root, labels_root)
        state["boxes"] = read_labels(target, frame.shape)
        if not state["boxes"] and detector is not None and not target.exists():
            state["boxes"] = [(KNOWN_LABELS.index(d.label), *d.bbox)
                              for d in detector.detect(frame) if d.label in KNOWN_LABELS]
        state["scale"] = min(1.0, MAX_SIDE / max(frame.shape[:2]))

        step = 0
        while step == 0:
            cv.imshow(WINDOW, draw(frame, state["boxes"], state["current"],
                                   image_path.name, index, len(images), state["scale"]))
            key = cv.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                write_labels(target, state["boxes"], frame.shape)
                cv.destroyAllWindows()
                print(f"stopped at {image_path}")
                return 0
            if key in (ord("1"), ord("2"), ord("3"), ord("4")):
                state["current"] = min(key - ord("1"), len(KNOWN_LABELS) - 1)
            elif key == ord("u") and state["boxes"]:
                state["boxes"].pop()
            elif key == ord("c"):
                state["boxes"] = []
            elif key == ord("s"):
                write_labels(target, state["boxes"], frame.shape)
            elif key == ord("n"):
                step = 1
            elif key == ord("p"):
                step = -1

        write_labels(target, state["boxes"], frame.shape)
        index += step

    cv.destroyAllWindows()
    written = len(list(Path(args.labels_root).rglob("*.txt")))
    print(f"done - {written} label files under {args.labels_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
