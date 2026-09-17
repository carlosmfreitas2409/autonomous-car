#!/usr/bin/env python3
"""Composite a YOLO training set out of sign cutouts and real backgrounds.

Compositing rather than hand-labelling, because the paste location *is* the
label: labelling costs nothing and box tightness is exact. What it buys in
volume it has to earn back in honesty, so three things here are not optional.

**Every scale.** Per-class, because one range cannot fit both: a ~50 cm light
housing is ~260 px at 2 m and ~520 px at 1 m, while a 15 cm delivery sign can
never exceed ~200 px in a 720p frame before the car has driven past it. The
most expensive mistake in this project's record was a 280 px cap on the light -
confidence measured 0.97 at 260 px, 0.18 at 400 px and 0.00 at 500 px, so the
housing vanished exactly where the car has to stop.

**Distractors that are deliberately not signs.** A model trained on clean
rooms has never seen an empty scene, concludes a sign is always present
somewhere, and picks whatever is most sign-like - which at an exhibition is a
stage lamp, a phone screen or a person. The record has a bare red disc called
`stop` at 0.86 and a green disc called `traffic_light` at 0.63. Half the frames
here carry unlabelled emitters, and one in six carries no sign at all.

**A box that is the sign face or the lamp housing and nothing else.** Never the
post. Phase-2 distance comes from real_height / pixel_height, which is only
constant if the box always bounds the same physical object, and the state
reader locates lamp cells inside the box, which only works if the box is the
housing.

    python3 tools/synthesize_dataset.py --count 4000 --preview 24

`degrade_frame`, `warp_cutout` and `render_onto` are imported by
tools/eval_delivery_variant.py: the thing measured and the thing trained on
have to be the same thing, so they are written once, here.
"""

import argparse
import random
import shutil
from pathlib import Path

import cv2 as cv
import numpy as np

# Class order. This is what ends up in the ONNX metadata, and
# src/perception/sign_detection.py's KNOWN_LABELS must match it.
CLASSES = ("delivery_sign", "detour", "stop", "traffic_light")

FRAME_SIZE = (1280, 720)

# Apparent height in pixels, per class. See the module docstring.
PASTE_HEIGHT = {
    "traffic_light": (22, 640),
    "delivery_sign": (22, 260),
    "detour": (22, 260),
    "stop": (22, 260),
}

NO_SIGN_SHARE = 1 / 6
DISTRACTOR_SHARE = 0.5
VAL_SHARE = 0.15


# --------------------------------------------------------------------------
# the pieces the eval imports too
# --------------------------------------------------------------------------

def warp_cutout(bgra: np.ndarray, rng: random.Random, extra: np.ndarray = None):
    """Rotate and perspective-warp a cutout, keeping its alpha exact.

    The car sees signs from the side and from below, never square on. The warp
    is applied to the alpha as well, so the box stays tight on whatever shape
    comes out. `extra` rides through the same transform: it carries the
    housing's own alpha for a lamp whose halo has grown the visible one.
    """
    height, width = bgra.shape[:2]
    corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    jitter = 0.16 * min(width, height)
    shifted = corners + np.float32([[rng.uniform(-jitter, jitter) for _ in range(2)]
                                    for _ in range(4)])
    shifted -= shifted.min(axis=0)
    out_w = max(8, int(np.ceil(shifted[:, 0].max())))
    out_h = max(8, int(np.ceil(shifted[:, 1].max())))
    matrix = cv.getPerspectiveTransform(corners, shifted)
    angle = rng.uniform(-8, 8)
    rotation = cv.getRotationMatrix2D((out_w / 2, out_h / 2), angle, 1.0)

    def apply(image, border):
        warped_image = cv.warpPerspective(image, matrix, (out_w, out_h),
                                          flags=cv.INTER_LINEAR, borderValue=border)
        return cv.warpAffine(warped_image, rotation, (out_w, out_h),
                             flags=cv.INTER_LINEAR, borderValue=border)

    warped = apply(bgra, (0, 0, 0, 0))
    if extra is None:
        return warped
    return warped, apply(extra, 0)


def bloom(bgra: np.ndarray, rng: random.Random) -> np.ndarray:
    """Blow out the bright part of a cutout and let it spill past its edge.

    This is what the camera does to a lit lamp: the core clips to white and the
    colour survives only in a halo outside the housing. A model trained on
    clean cutouts has never seen it and misses every lit light - which is
    exactly what the previous model did, scoring 0.000-0.005 on the three
    photos where a lamp was on.
    """
    strength = rng.uniform(0.35, 1.0)
    bgr = bgra[:, :, :3].astype(np.float32)
    alpha = bgra[:, :, 3].astype(np.float32) / 255.0
    luma = bgr.max(axis=2) * alpha
    hot = np.clip((luma - 205) / 50.0, 0, 1)
    if hot.max() <= 0.02:
        return bgra
    radius = max(3, int(0.08 * max(bgra.shape[:2])) | 1)
    halo = cv.GaussianBlur(hot, (radius, radius), 0)
    glow = np.clip(halo * strength, 0, 1)[:, :, None]
    tint = bgr * (1 - glow) + 255.0 * glow
    out = bgra.copy()
    out[:, :, :3] = np.clip(tint + hot[:, :, None] * 60 * strength, 0, 255).astype(np.uint8)
    # The halo reaches past the housing, so the alpha grows with it.
    out[:, :, 3] = np.clip(alpha * 255 + halo * 200 * strength, 0, 255).astype(np.uint8)
    return out


def render_onto(frame: np.ndarray, bgra: np.ndarray, height: int,
                rng: random.Random, position=None, box_alpha: np.ndarray = None):
    """Warp a cutout to an apparent height and paste it. Returns its box.

    The box covers the sign face or the lamp housing - never the post, because
    the artwork has none and the light cutouts were cut at the housing, and
    never the halo either. A bloomed lamp spills light several radii past its
    housing; if the box followed that, the housing's apparent height would
    change with how bright the lamp is, and phase-2 distance is
    real_height / pixel_height. `box_alpha` is the housing's own alpha, which
    the halo never touches.
    """
    if box_alpha is None:
        box_alpha = bgra[:, :, 3]
    warped, warped_box_alpha = warp_cutout(bgra, rng, extra=box_alpha)
    scale = height / warped.shape[0]
    target = (max(4, int(round(warped.shape[1] * scale))), max(4, height))
    interpolation = cv.INTER_AREA if scale < 1 else cv.INTER_LINEAR
    warped = cv.resize(warped, target, interpolation=interpolation)
    warped_box_alpha = cv.resize(warped_box_alpha, target, interpolation=interpolation)

    frame_h, frame_w = frame.shape[:2]
    sign_h, sign_w = warped.shape[:2]
    if position is None:
        x = rng.randint(-sign_w // 3, frame_w - sign_w + sign_w // 3)
        y = rng.randint(-sign_h // 3, frame_h - sign_h + sign_h // 3)
    else:
        x, y = position

    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(frame_w, x + sign_w), min(frame_h, y + sign_h)
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None
    patch = warped[y0 - y:y1 - y, x0 - x:x1 - x]
    alpha = patch[:, :, 3:].astype(np.float32) / 255.0
    region = frame[y0:y1, x0:x1].astype(np.float32)
    frame[y0:y1, x0:x1] = (region * (1 - alpha) + patch[:, :, :3] * alpha).astype(np.uint8)

    solid = np.argwhere(warped_box_alpha[y0 - y:y1 - y, x0 - x:x1 - x] > 40)
    if solid.size == 0:
        return None
    (top, left), (bottom, right) = solid.min(axis=0), solid.max(axis=0)
    return (x0 + int(left), y0 + int(top), x0 + int(right) + 1, y0 + int(bottom) + 1)


def degrade_frame(frame: np.ndarray, rng: random.Random) -> np.ndarray:
    """Everything between the scene and the file: optics, sensor, codec.

    Exposure and gamma first because the venue is dark and the car's camera
    will push it, then blur - motion or defocus, not both - then sensor noise,
    then JPEG. In that order, because that is the order a camera does it in.
    """
    out = frame.astype(np.float32)

    gain = rng.uniform(0.55, 1.45)
    gamma = rng.uniform(0.75, 1.35)
    out = 255.0 * np.power(np.clip(out * gain / 255.0, 0, 1), gamma)

    # White balance drifts: the hall is warm, daylight is not.
    cast = np.float32([rng.uniform(0.9, 1.12) for _ in range(3)])
    out = np.clip(out * cast, 0, 255)

    choice = rng.random()
    if choice < 0.35:
        length = rng.randint(3, 15)
        angle = rng.uniform(0, np.pi)
        kernel = np.zeros((length, length), np.float32)
        cv.line(kernel,
                (int((length - 1) / 2 * (1 - np.cos(angle))), int((length - 1) / 2 * (1 - np.sin(angle)))),
                (int((length - 1) / 2 * (1 + np.cos(angle))), int((length - 1) / 2 * (1 + np.sin(angle)))),
                1.0, 1)
        total = kernel.sum()
        if total > 0:
            out = cv.filter2D(out, -1, kernel / total)
    elif choice < 0.65:
        radius = rng.choice((3, 5, 7))
        out = cv.GaussianBlur(out, (radius, radius), 0)

    out += np.random.normal(0, rng.uniform(1.5, 9.0), out.shape)
    out = np.clip(out, 0, 255).astype(np.uint8)

    quality = rng.randint(38, 92)
    ok, buffer = cv.imencode(".jpg", out, [cv.IMWRITE_JPEG_QUALITY, quality])
    return cv.imdecode(buffer, cv.IMREAD_COLOR) if ok else out


def draw_distractors(frame: np.ndarray, rng: random.Random) -> None:
    """Bright things that are emphatically not signs, and carry no label.

    A vertical stack of three lamps is never drawn: that is the traffic
    light's own silhouette, and teaching the model it is a negative would
    teach it to miss the thing it exists to find. Pairs are drawn, because a
    pair is what a stage rig actually looks like.
    """
    frame_h, frame_w = frame.shape[:2]
    for _ in range(rng.randint(1, 4)):
        kind = rng.random()
        colour = rng.choice([(40, 40, 255), (40, 220, 255), (60, 230, 90),
                             (255, 255, 255), (255, 190, 120)])
        if kind < 0.45:                                    # a lamp, or two
            radius = rng.randint(6, 90)
            cx, cy = rng.randint(0, frame_w), rng.randint(0, frame_h)
            count = rng.choice((1, 1, 2))
            step = rng.randint(2, 4) * radius
            for index in range(count):
                _glowing_disc(frame, cx, cy + index * step, radius, colour, rng)
        elif kind < 0.7:                                   # an LED bar
            width = rng.randint(20, 240)
            height = max(3, int(width * rng.uniform(0.04, 0.25)))
            x, y = rng.randint(0, frame_w), rng.randint(0, frame_h)
            overlay = frame.copy()
            cv.rectangle(overlay, (x, y), (x + width, y + height), colour, -1)
            cv.addWeighted(overlay, rng.uniform(0.5, 1.0), frame, rng.uniform(0.0, 0.5), 0, frame)
        else:                                              # a screen
            width = rng.randint(30, 260)
            height = int(width * rng.uniform(0.5, 1.8))
            x, y = rng.randint(0, frame_w), rng.randint(0, frame_h)
            panel = np.full((height, width, 3), rng.randint(90, 255), np.uint8)
            for _ in range(rng.randint(1, 5)):
                cv.rectangle(panel, (rng.randint(0, width), rng.randint(0, height)),
                             (rng.randint(0, width), rng.randint(0, height)),
                             tuple(rng.randint(0, 255) for _ in range(3)), -1)
            x1, y1 = min(frame_w, x + width), min(frame_h, y + height)
            if x1 > x and y1 > y:
                frame[y:y1, x:x1] = panel[:y1 - y, :x1 - x]


def _glowing_disc(frame, cx, cy, radius, colour, rng):
    layer = np.zeros(frame.shape[:2], np.float32)
    cv.circle(layer, (cx, cy), radius, 1.0, -1)
    blur = max(3, int(radius * rng.uniform(0.8, 2.2)) | 1)
    halo = cv.GaussianBlur(layer, (blur, blur), 0)
    halo = np.clip(layer + halo * rng.uniform(0.6, 1.4), 0, 1)[:, :, None]
    tint = np.float32(colour)[None, None, :]
    core = np.clip(layer, 0, 1)[:, :, None] * rng.uniform(0.5, 1.0)
    blended = frame.astype(np.float32) * (1 - halo) + tint * halo
    frame[:] = np.clip(blended * (1 - core) + 255.0 * core, 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------
# the dataset
# --------------------------------------------------------------------------

def load_cutouts(root: Path) -> dict:
    cutouts = {}
    for label in CLASSES:
        files = sorted((root / label).glob("*.png"))
        loaded = []
        for path in files:
            image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
            if image is None or image.shape[2] != 4:
                print(f"  skipping {path}: needs an alpha channel")
                continue
            loaded.append((path.stem, image))
        if not loaded:
            print(f"  no cutouts for {label} under {root / label}")
        cutouts[label] = loaded
    return cutouts


def load_backgrounds(directories) -> list:
    paths = []
    for directory in directories:
        directory = Path(directory)
        if not directory.exists():
            continue
        paths += [p for p in sorted(directory.rglob("*"))
                  if p.suffix.lower() in (".jpg", ".jpeg", ".png")]
    return paths


def background_frame(path: Path, rng: random.Random) -> np.ndarray:
    image = cv.imread(str(path))
    if image is None:
        return np.full((FRAME_SIZE[1], FRAME_SIZE[0], 3), 40, np.uint8)
    # A random crop, so 519 frames are not 519 identical framings.
    height, width = image.shape[:2]
    scale = rng.uniform(0.6, 1.0)
    crop_w, crop_h = int(width * scale), int(height * scale)
    x = rng.randint(0, width - crop_w)
    y = rng.randint(0, height - crop_h)
    cropped = image[y:y + crop_h, x:x + crop_w]
    frame = cv.resize(cropped, FRAME_SIZE)
    return cv.flip(frame, 1) if rng.random() < 0.5 else frame


def make_sample(frame, cutouts, rng):
    """Paste zero or one sign and return its YOLO line, if any."""
    if rng.random() < NO_SIGN_SHARE:
        return None
    label = rng.choice([c for c in CLASSES if cutouts[c]])
    name, cutout = rng.choice(cutouts[label])
    low, high = PASTE_HEIGHT[label]
    # Uniform in log height: without it, three quarters of the samples land in
    # the top octave and the far-away sizes never get trained.
    height = int(round(np.exp(rng.uniform(np.log(low), np.log(high)))))
    housing_alpha = cutout[:, :, 3].copy()
    if label == "traffic_light" and rng.random() < 0.7:
        cutout = bloom(cutout, rng)
    box = render_onto(frame, cutout, height, rng, box_alpha=housing_alpha)
    if box is None:
        return None
    return label, box, name


def yolo_line(label: str, box, shape) -> str:
    height, width = shape[:2]
    x0, y0, x1, y1 = box
    cx, cy = (x0 + x1) / 2 / width, (y0 + y1) / 2 / height
    return f"{CLASSES.index(label)} {cx:.6f} {cy:.6f} {(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--count", type=int, default=4000)
    parser.add_argument("--out", default="dataset/synth")
    parser.add_argument("--cutouts", default="assets/sign_cutouts")
    parser.add_argument("--backgrounds", nargs="+", default=["dataset/backgrounds"])
    parser.add_argument("--preview", type=int, default=0,
                        help="write this many boxed previews and stop before training")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)

    cutouts = load_cutouts(Path(args.cutouts))
    backgrounds = load_backgrounds(args.backgrounds)
    if not backgrounds:
        print(f"no backgrounds under {args.backgrounds}")
        return 2
    print(f"{sum(len(v) for v in cutouts.values())} cutouts, {len(backgrounds)} backgrounds")

    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True)
        (out / "labels" / split).mkdir(parents=True)
    preview_dir = out / "preview"
    if args.preview:
        preview_dir.mkdir()

    counts = {label: 0 for label in CLASSES}
    empty = 0
    for index in range(args.count):
        split = "val" if rng.random() < VAL_SHARE else "train"
        frame = background_frame(rng.choice(backgrounds), rng)
        # Distractors first: they are clutter in the scene, and drawing them
        # last would occasionally paint over the sign while its label stayed.
        if rng.random() < DISTRACTOR_SHARE:
            draw_distractors(frame, rng)
        sample = make_sample(frame, cutouts, rng)
        frame = degrade_frame(frame, rng)

        lines = []
        if sample is not None:
            label, box, _ = sample
            lines.append(yolo_line(label, box, frame.shape))
            counts[label] += 1
        else:
            empty += 1

        name = f"{index:06d}"
        cv.imwrite(str(out / "images" / split / f"{name}.jpg"), frame)
        # Empty files on purpose: an image with no sign is a negative, and
        # ultralytics only counts it as one if the label file exists.
        (out / "labels" / split / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))

        if args.preview and index < args.preview:
            preview = frame.copy()
            for line in lines:
                parts = line.split()
                cx, cy, w, h = (float(v) for v in parts[1:])
                fh, fw = frame.shape[:2]
                cv.rectangle(preview,
                             (int((cx - w / 2) * fw), int((cy - h / 2) * fh)),
                             (int((cx + w / 2) * fw), int((cy + h / 2) * fh)), (0, 255, 0), 2)
                cv.putText(preview, CLASSES[int(parts[0])],
                           (int((cx - w / 2) * fw), max(14, int((cy - h / 2) * fh) - 6)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv.imwrite(str(preview_dir / f"{name}.jpg"), preview)

    (out / "data.yaml").write_text(
        f"path: {out.resolve()}\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(CLASSES)}\n"
        f"names: {list(CLASSES)}\n"
    )
    print(f"{args.count} images -> {out}")
    print(f"  per class: {counts}")
    print(f"  no sign at all: {empty}")
    if args.preview:
        print(f"\nLook at {preview_dir} before training. If the boxes are not tight")
        print("on the signs, nothing downstream matters.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
