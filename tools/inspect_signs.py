#!/usr/bin/env python3
"""Watch the detector and both crop readers work, one window, no web.

The point of this tool is that a suppressed false positive is *visible as a
rejection* rather than silently absent: anything that fails structural
verification is drawn thin and grey with its reason next to it. A detector you
can only see the survivors of is a detector you cannot debug.

    python3 tools/inspect_signs.py --images 'dataset/real/images/**/*.jpg'
    python3 tools/inspect_signs.py --video clips/light_approach.mp4
    python3 tools/inspect_signs.py --camera 1
    python3 tools/inspect_signs.py --images a.jpg --box 400,60,520,340   # no detector at all

`--box` drives the readers with no model involved, which is where this project
is on any day the detector is broken - and the record says that is most days.

Keys
    q / ESC   quit              SPACE   pause
    . / ,     step a frame      f / d   confidence +/- 0.05      0  reset it
    b         cycle which detections the inspector column shows
    m         mask overlay on the main pane
    s         save the canvas   h       this help
"""

import argparse
import glob
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from perception.delivery_variant import CROP_MARGIN as VARIANT_MARGIN  # noqa: E402
from perception.delivery_variant import read_delivery_variant  # noqa: E402
from perception.sign_detection import Detection, SignDetector  # noqa: E402
from perception.traffic_light import CROP_MARGIN as LIGHT_MARGIN  # noqa: E402
from perception.traffic_light import LightState, read_light_state  # noqa: E402

WINDOW = "inspect_signs"
PANEL_WIDTH = 320
TILE = 96
MAX_SHOWN = 3
MAIN_WIDTH = 960

STATE_COLOUR = {
    LightState.RED: (60, 60, 235),
    LightState.YELLOW: (40, 210, 235),
    LightState.GREEN: (80, 210, 90),
    LightState.UNKNOWN: (150, 150, 150),
}
CLASS_COLOUR = {
    "delivery_sign": (235, 190, 90),
    "detour": (60, 200, 245),
    "stop": (70, 70, 240),
    "traffic_light": (200, 200, 200),
}
GREY = (110, 110, 110)


class Reading:
    """One detection plus whatever its crop reader made of it."""

    def __init__(self, detection: Detection, frame: np.ndarray):
        self.detection = detection
        self.light = None
        self.variant = None
        if detection.label == "traffic_light":
            self.light = read_light_state(detection.crop(frame, LIGHT_MARGIN))
        elif detection.label == "delivery_sign":
            self.variant = read_delivery_variant(detection.crop(frame, VARIANT_MARGIN))

    @property
    def verified(self) -> bool:
        """Did the crop back the detector up? Unverified is drawn as rejected."""
        if self.light is not None:
            return self.light.verified
        return True

    @property
    def colour(self):
        if not self.verified:
            return GREY
        if self.light is not None:
            return STATE_COLOUR[self.light.state]
        return CLASS_COLOUR.get(self.detection.label, (200, 200, 200))

    @property
    def caption(self) -> str:
        label, confidence = self.detection.label, self.detection.confidence
        if self.light is not None:
            if not self.light.verified:
                return f"{label} {confidence:.2f} -> rejected: {self.light.reason}"
            return (f"{label} {confidence:.2f} -> {self.light.state.value} "
                    f"{self.light.confidence:.2f}")
        if self.variant is not None:
            named = self.variant.variant or "?"
            return f"{label} {confidence:.2f} -> {named} {self.variant.confidence:.2f}"
        aspect = self.detection.height / max(1, self.detection.width)
        return f"{label} {confidence:.2f} ar {aspect:.2f}"

    def tiles(self, frame) -> list:
        if self.light is not None:
            debug = self.light.debug
            if debug:
                return [debug["work"],
                        cv.cvtColor(debug["core_mask"], cv.COLOR_GRAY2BGR),
                        cv.cvtColor(debug["halo_mask"], cv.COLOR_GRAY2BGR)]
            return [self.detection.crop(frame, LIGHT_MARGIN)]
        if self.variant is not None:
            debug = self.variant.debug
            tiles = [debug.get("work", self.detection.crop(frame, VARIANT_MARGIN))]
            face = debug.get("face_mask")
            if face is not None:
                overlay = cv.cvtColor(face, cv.COLOR_GRAY2BGR)
                contour = debug.get("contour")
                if contour is not None:
                    cv.drawContours(overlay, [contour], -1, (60, 200, 245), 1)
                tiles.append(overlay)
            glyph = debug.get("glyph")
            if glyph is not None:
                normalised = cv.normalize(glyph, None, 0, 255, cv.NORM_MINMAX)
                tiles.append(cv.cvtColor(normalised.astype(np.uint8), cv.COLOR_GRAY2BGR))
            return tiles
        return [self.detection.crop(frame)]

    def lines(self) -> list:
        if self.light is not None:
            reading = self.light
            if reading.cells is None:
                return [reading.reason]
            cells = reading.cells
            return [
                f"{reading.state.value} conf {reading.confidence:.2f} | "
                f"cells {cells.found}/3 r {cells.radius:.0f}",
                f"pos {reading.position_evidence:.2f} | hue {reading.colour_evidence:.2f} "
                f"-> {reading.reason}",
            ]
        if self.variant is not None:
            reading = self.variant
            if reading.sides is None and reading.holes is None:
                return [reading.reason]
            sides = (f"sides {reading.sides} -> {reading.sides_variant} "
                     f"(harmonic {reading.sides_margin:.1f}x)")
            letter = (f"letter {reading.letter_variant} "
                      f"(holes {reading.holes}, gap {reading.letter_gap:.2f})")
            return [sides, letter, f"=> {reading.variant or 'unknown'}: {reading.reason}"]
        return [self.caption]


def frames_from(args):
    """Whatever source was asked for, as a stream of (name, frame)."""
    if args.images:
        paths = []
        for pattern in args.images:
            paths += sorted(glob.glob(pattern, recursive=True))
        if not paths:
            print(f"no images matched {args.images}")
        for path in paths:
            frame = cv.imread(path)
            if frame is not None:
                yield Path(path).name, frame
        return

    source = args.video if args.video else args.camera
    capture = cv.VideoCapture(source)
    if not capture.isOpened():
        print(f"could not open {source}")
        return
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        yield f"frame {index}", frame
        index += 1
    capture.release()


def fit(image, width, height):
    if image is None or image.size == 0:
        return np.zeros((height, width, 3), np.uint8)
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv.resize(image, (max(1, int(image.shape[1] * scale)),
                                max(1, int(image.shape[0] * scale))))
    canvas = np.zeros((height, width, 3), np.uint8)
    y = (height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return canvas


def render(frame, readings, name, confidence, available, overlay, offset):
    main = frame.copy()
    for reading in readings:
        x0, y0, x1, y1 = reading.detection.bbox
        thickness = 1 if not reading.verified else 2
        cv.rectangle(main, (x0, y0), (x1, y1), reading.colour, thickness)
        cv.putText(main, reading.caption, (x0, max(14, y0 - 6)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, reading.colour, 1)
        if overlay and reading.light is not None and reading.light.cells is not None:
            cells = reading.light.cells
            for index, cy in enumerate(cells.centres):
                if index in cells.clipped:
                    continue
                cv.circle(main, (int(x0 + cells.cx - (x1 - x0) * LIGHT_MARGIN),
                                 int(y0 + cy - (y1 - y0) * LIGHT_MARGIN)),
                          int(cells.radius), reading.colour, 1)

    scale = MAIN_WIDTH / main.shape[1]
    main = cv.resize(main, (MAIN_WIDTH, int(main.shape[0] * scale)))

    panel = np.zeros((main.shape[0], PANEL_WIDTH, 3), np.uint8)
    shown = readings[offset:offset + MAX_SHOWN] if readings else []
    y = 4
    for reading in shown:
        tiles = reading.tiles(frame)[:3]
        for column, tile in enumerate(tiles):
            panel[y:y + TILE, column * (TILE + 4):column * (TILE + 4) + TILE] = \
                fit(tile, TILE, TILE)
        y += TILE + 4
        for line in reading.lines()[:3]:
            cv.putText(panel, line[:46], (4, y + 10), cv.FONT_HERSHEY_SIMPLEX, 0.35,
                       reading.colour, 1)
            y += 14
        y += 8
    if not shown:
        cv.putText(panel, "nothing detected", (8, 20), cv.FONT_HERSHEY_SIMPLEX, 0.45, GREY, 1)

    canvas = np.hstack([main, panel])
    header = f"{name}   conf {confidence:.2f}   {len(readings)} detection(s)"
    cv.rectangle(canvas, (0, 0), (canvas.shape[1], 22), (0, 0, 0), -1)
    cv.putText(canvas, header, (6, 16), cv.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1)
    if not available:
        cv.rectangle(canvas, (0, 22), (canvas.shape[1], 44), (0, 0, 160), -1)
        cv.putText(canvas, "no model - frames only", (6, 38),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--images", nargs="+")
    source.add_argument("--video")
    source.add_argument("--camera", type=int, default=0)
    parser.add_argument("--model", default="models/signs.onnx")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--box", help="x0,y0,x1,y1 - drive the readers with no detector")
    parser.add_argument("--box-label", default="traffic_light")
    parser.add_argument("--save-dir", default=".")
    args = parser.parse_args()

    detector = SignDetector(args.model, confidence=args.confidence)
    if not detector.available:
        print(f"no usable model at {args.model} - frames and --box only")

    fixed = None
    if args.box:
        parts = [int(v) for v in args.box.split(",")]
        if len(parts) != 4:
            print("--box wants x0,y0,x1,y1")
            return 2
        fixed = Detection(args.box_label, tuple(parts), 1.0)

    confidence = args.confidence
    paused = False
    overlay = False
    offset = 0
    saved = 0

    stream = frames_from(args)
    current = next(stream, None)
    if current is None:
        print("nothing to show")
        return 2

    while current is not None:
        name, frame = current
        detections = [fixed] if fixed else detector.detect(frame)
        readings = [Reading(d, frame) for d in detections]
        canvas = render(frame, readings, name, confidence, detector.available or bool(fixed),
                        overlay, offset)
        cv.imshow(WINDOW, canvas)

        step = not paused
        while True:
            key = cv.waitKey(0 if paused else 20) & 0xFF
            if key == 255:
                break
            if key in (ord("q"), 27):
                cv.destroyAllWindows()
                return 0
            if key == ord(" "):
                paused = not paused
                break
            if key in (ord("."), ord(",")):
                paused = True
                step = True
                break
            if key in (ord("f"), ord("d")):
                confidence = float(np.clip(confidence + (0.05 if key == ord("f") else -0.05),
                                           0.05, 0.95))
                detector.set_confidence(confidence)
                break
            if key == ord("0"):
                confidence = args.confidence
                detector.set_confidence(confidence)
                break
            if key == ord("b"):
                offset = (offset + MAX_SHOWN) % max(1, len(readings))
                break
            if key == ord("m"):
                overlay = not overlay
                break
            if key == ord("s"):
                path = Path(args.save_dir) / f"inspect_{saved:03d}.png"
                cv.imwrite(str(path), canvas)
                print(f"saved {path}")
                saved += 1
                break
            if key == ord("h"):
                print(__doc__)
                break
            break

        if step:
            current = next(stream, None)
            offset = 0

    cv.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
