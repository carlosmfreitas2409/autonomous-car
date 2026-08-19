"""Build a labelled sign dataset by compositing artwork onto real track frames.

Hand-labelling a few thousand images is the step that usually kills a project
like this. It is also unnecessary here: the signs are rigid, flat, and we have
the artwork, so we can paste them onto real background frames ourselves. Where
we pasted it *is* the label, so the annotation is exact and free.

    assets/sign_cutouts/stop/*.png           RGBA, tight around the sign
    assets/sign_cutouts/detour/*.png
    assets/sign_cutouts/traffic_light/*.png
    dataset/backgrounds/*.jpg                from tools/capture_dataset.py

    python3 tools/synthesize_dataset.py --count 3000
    python3 tools/synthesize_dataset.py --count 3000 --preview 12

Class names come from the cutout directory names and are written into
data.yaml, which ultralytics embeds into the exported ONNX, which the car
reads back at load time. Nothing about the class list is hardcoded in src/.

One caveat worth stating plainly: the val split this writes is still
synthetic, so it will flatter itself. tools/eval_signs.py exists to score the
model against real held-out photos, which is the number to actually trust.
"""

import argparse
import math
import random
from pathlib import Path

import cv2 as cv
import numpy as np


CUTOUT_ROOT = Path("assets/sign_cutouts")
BACKGROUND_ROOT = Path("dataset/backgrounds")
OUTPUT_ROOT = Path("dataset/synth")

OUTPUT_WIDTH = 1280
OUTPUT_HEIGHT = 720

# Pasted sign height in pixels. The model sees 640 px, so a sign in a 1280 px
# frame is halved on the way in: 22 px here is the ~11 px the detector will
# barely manage, and 280 px is a sign filling the view. Sampling log-uniformly
# between them spends most of the budget on the small, hard, distant cases.
MIN_SIGN_PX = 22
MAX_SIGN_PX = 280

# Cutouts are scaled down to this on load. Comfortably above the largest paste,
# and far cheaper to warp than a full-resolution photo.
WORKING_SIGN_PX = 600

# Signs sit at roughly camera height, so they land in this vertical band.
VERTICAL_BAND = (0.15, 0.80)

MAX_ROTATION_DEG = 12.0
MAX_PERSPECTIVE = 0.06
MAX_OVERLAP_IOU = 0.15
ALPHA_CUTOFF = 16


def load_cutouts(root: Path) -> dict[str, list[np.ndarray]]:
    cutouts: dict[str, list[np.ndarray]] = {}
    for class_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        images = []
        for path in sorted(class_dir.glob("*.png")):
            image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
            if image is None:
                continue
            if image.shape[2] == 3:
                print(f"  skipping {path} - needs an alpha channel")
                continue
            # Cutouts get pasted at up to MAX_SIGN_PX tall. Anything smaller
            # has to be scaled up, and an upscaled sign is soft in a way no
            # real camera image is - the model learns the blur, not the sign.
            if image.shape[0] < MAX_SIGN_PX:
                print(f"  {path.name} is only {image.shape[0]} px tall; "
                      f"export at {MAX_SIGN_PX}+ (512 is a good default)")
            # Each paste is padded and warped twice at the cutout's own
            # resolution, so an oversized source costs real time for detail
            # that is thrown away on the way down to MAX_SIGN_PX anyway.
            if image.shape[0] > WORKING_SIGN_PX:
                scale = WORKING_SIGN_PX / image.shape[0]
                image = cv.resize(image, (max(1, int(image.shape[1] * scale)), WORKING_SIGN_PX),
                                  interpolation=cv.INTER_AREA)
            images.append(image)
        if images:
            cutouts[class_dir.name] = images
            print(f"  {class_dir.name}: {len(images)} cutout(s)")
    return cutouts


def crop_to_alpha(rgba: np.ndarray) -> np.ndarray:
    ys, xs = np.nonzero(rgba[:, :, 3] > ALPHA_CUTOFF)
    if ys.size == 0:
        return rgba
    return rgba[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def warp(rgba: np.ndarray) -> np.ndarray:
    """Rotate and tilt the sign, as though seen from off to one side."""
    pad = int(max(rgba.shape[:2]) * 0.35)
    padded = cv.copyMakeBorder(rgba, pad, pad, pad, pad, cv.BORDER_CONSTANT, value=(0, 0, 0, 0))
    height, width = padded.shape[:2]

    angle = random.uniform(-MAX_ROTATION_DEG, MAX_ROTATION_DEG)
    rotation = cv.getRotationMatrix2D((width / 2, height / 2), angle, 1.0)
    rotated = cv.warpAffine(padded, rotation, (width, height), borderValue=(0, 0, 0, 0))

    corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    jitter = MAX_PERSPECTIVE * min(width, height)
    shifted = corners + np.random.uniform(-jitter, jitter, corners.shape).astype(np.float32)
    transform = cv.getPerspectiveTransform(corners, shifted)
    tilted = cv.warpPerspective(rotated, transform, (width, height), borderValue=(0, 0, 0, 0))

    return crop_to_alpha(tilted)


def recolour(rgba: np.ndarray) -> np.ndarray:
    """Independent channel gains: sunlight, shade, and white balance drift."""
    gains = np.array([random.uniform(0.80, 1.20) for _ in range(3)], dtype=np.float32)
    gains *= random.uniform(0.55, 1.25)
    out = rgba.copy()
    out[:, :, :3] = np.clip(out[:, :, :3].astype(np.float32) * gains, 0, 255).astype(np.uint8)
    return out


def occlude(rgba: np.ndarray) -> np.ndarray:
    """Hide part of the sign. The label still covers the whole thing, which is
    what teaches the detector to report the full extent of a partly hidden sign."""
    out = rgba.copy()
    height, width = out.shape[:2]
    fraction = random.uniform(0.12, 0.35)
    if random.random() < 0.5:
        cut = int(width * fraction)
        columns = slice(0, cut) if random.random() < 0.5 else slice(width - cut, width)
        out[:, columns, 3] = 0
    else:
        cut = int(height * fraction)
        rows = slice(0, cut) if random.random() < 0.5 else slice(height - cut, height)
        out[rows, :, 3] = 0
    return out


def paste(background: np.ndarray, rgba: np.ndarray, x: int, y: int) -> None:
    height, width = rgba.shape[:2]
    region = background[y:y + height, x:x + width]
    alpha = (rgba[:, :, 3:4].astype(np.float32) / 255.0)
    blended = rgba[:, :, :3].astype(np.float32) * alpha + region.astype(np.float32) * (1.0 - alpha)
    background[y:y + height, x:x + width] = blended.astype(np.uint8)


def iou(a, b) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    inter_w = min(ax1, bx1) - max(ax0, bx0)
    inter_h = min(ay1, by1) - max(ay0, by0)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0
    intersection = inter_w * inter_h
    union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - intersection
    return intersection / union if union > 0 else 0.0


def motion_blur(image: np.ndarray) -> np.ndarray:
    length = random.randint(3, 15)
    kernel = np.zeros((length, length), dtype=np.float32)
    kernel[length // 2, :] = 1.0
    rotation = cv.getRotationMatrix2D((length / 2 - 0.5, length / 2 - 0.5), random.uniform(0, 180), 1.0)
    kernel = cv.warpAffine(kernel, rotation, (length, length))
    total = kernel.sum()
    return cv.filter2D(image, -1, kernel / total) if total > 0 else image


def degrade(image: np.ndarray) -> np.ndarray:
    """Everything between the scene and the model: shake, sensor noise, JPEG."""
    if random.random() < 0.35:
        image = motion_blur(image)
    if random.random() < 0.4:
        noise = np.random.normal(0, random.uniform(2, 8), image.shape).astype(np.float32)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    if random.random() < 0.5:
        quality = random.randint(55, 95)
        ok, encoded = cv.imencode(".jpg", image, [int(cv.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            image = cv.imdecode(encoded, cv.IMREAD_COLOR)
    return image


def compose(background: np.ndarray, cutouts: dict, class_names: list[str], sign_count: int):
    canvas = cv.resize(background, (OUTPUT_WIDTH, OUTPUT_HEIGHT))
    boxes = []

    for _ in range(sign_count):
        class_name = random.choice(list(cutouts.keys()))
        sign = warp(recolour(random.choice(cutouts[class_name])))

        target_h = int(math.exp(random.uniform(math.log(MIN_SIGN_PX), math.log(MAX_SIGN_PX))))
        scale = target_h / sign.shape[0]
        target_w = max(1, int(sign.shape[1] * scale))
        if target_w >= OUTPUT_WIDTH or target_h >= OUTPUT_HEIGHT:
            continue
        # INTER_AREA is the right filter going down and a poor one going up.
        shrinking = scale < 1.0
        sign = cv.resize(sign, (target_w, target_h),
                         interpolation=cv.INTER_AREA if shrinking else cv.INTER_CUBIC)

        placed = None
        for _ in range(10):
            x = random.randint(0, OUTPUT_WIDTH - target_w - 1)
            y = random.randint(
                int(OUTPUT_HEIGHT * VERTICAL_BAND[0]),
                max(int(OUTPUT_HEIGHT * VERTICAL_BAND[1]) - target_h, int(OUTPUT_HEIGHT * VERTICAL_BAND[0])),
            )
            candidate = (x, y, x + target_w, y + target_h)
            if all(iou(candidate, existing[1]) < MAX_OVERLAP_IOU for existing in boxes):
                placed = candidate
                break
        if placed is None:
            continue

        # The box is recorded before any occlusion, so a partly hidden sign is
        # still labelled at its true full extent.
        boxes.append((class_names.index(class_name), placed))
        paste(canvas, occlude(sign) if random.random() < 0.25 else sign, placed[0], placed[1])

    return degrade(canvas), boxes


def write_sample(image, boxes, split: str, index: int, root: Path) -> None:
    image_path = root / "images" / split / f"{index:06d}.jpg"
    label_path = root / "labels" / split / f"{index:06d}.txt"
    cv.imwrite(str(image_path), image, [int(cv.IMWRITE_JPEG_QUALITY), 92])

    lines = []
    for class_id, (x0, y0, x1, y1) in boxes:
        cx = (x0 + x1) / 2 / OUTPUT_WIDTH
        cy = (y0 + y1) / 2 / OUTPUT_HEIGHT
        w = (x1 - x0) / OUTPUT_WIDTH
        h = (y1 - y0) / OUTPUT_HEIGHT
        lines.append(f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    label_path.write_text("\n".join(lines) + ("\n" if lines else ""))


def write_preview(image, boxes, class_names, path: Path) -> None:
    annotated = image.copy()
    for class_id, (x0, y0, x1, y1) in boxes:
        cv.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv.putText(annotated, class_names[class_id], (x0, max(14, y0 - 5)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv.LINE_AA)
    cv.imwrite(str(path), annotated)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--count", type=int, default=3000)
    parser.add_argument("--val-fraction", type=float, default=0.1)
    parser.add_argument("--max-signs", type=int, default=3, help="most signs per image")
    parser.add_argument("--negative-fraction", type=float, default=0.15,
                        help="share of images containing no sign at all")
    parser.add_argument("--preview", type=int, default=0, help="also write N label-annotated samples")
    parser.add_argument("--cutouts", type=Path, default=CUTOUT_ROOT)
    parser.add_argument("--backgrounds", type=Path, default=BACKGROUND_ROOT)
    parser.add_argument("--output", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    if not args.cutouts.is_dir():
        raise SystemExit(
            f"No cutouts at {args.cutouts}.\n"
            "Create one directory per class holding RGBA PNGs of the sign artwork,\n"
            "for example assets/sign_cutouts/stop/stop.png"
        )

    print(f"Loading cutouts from {args.cutouts}")
    cutouts = load_cutouts(args.cutouts)
    if not cutouts:
        raise SystemExit("No usable RGBA cutouts found")

    backgrounds = sorted(args.backgrounds.glob("*.jpg")) + sorted(args.backgrounds.glob("*.png"))
    if not backgrounds:
        raise SystemExit(
            f"No backgrounds in {args.backgrounds}.\n"
            "Record some first: python3 tools/capture_dataset.py --auto"
        )
    print(f"{len(backgrounds)} background frame(s)")

    class_names = sorted(cutouts.keys())
    for split in ("train", "val"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)
    preview_dir = args.output / "preview"
    if args.preview:
        preview_dir.mkdir(parents=True, exist_ok=True)

    val_start = int(args.count * (1.0 - args.val_fraction))
    negatives = 0
    for index in range(args.count):
        background = cv.imread(str(random.choice(backgrounds)))
        if background is None:
            continue

        # Some images get no sign at all, and an empty label file. Without
        # these the model never sees a scene that simply has nothing in it, so
        # it learns that a sign is always present somewhere and picks whatever
        # is most sign-like — a face, a phone screen, a doorway.
        negative = random.random() < args.negative_fraction
        negatives += negative
        sign_count = 0 if negative else random.randint(1, args.max_signs)

        image, boxes = compose(background, cutouts, class_names, sign_count)
        write_sample(image, boxes, "train" if index < val_start else "val", index, args.output)
        if index < args.preview:
            write_preview(image, boxes, class_names, preview_dir / f"{index:06d}.jpg")
        if (index + 1) % 250 == 0:
            print(f"  {index + 1}/{args.count}")

    names_block = "\n".join(f"  {i}: {name}" for i, name in enumerate(class_names))
    (args.output / "data.yaml").write_text(
        f"path: {args.output.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n\n"
        f"names:\n{names_block}\n"
    )

    print(f"\nWrote {args.count} images to {args.output} ({negatives} with no sign)")
    print(f"Classes: {', '.join(class_names)}")
    if args.preview:
        print(f"Check the labels line up: {preview_dir}")
    print(f"\nNext: python3 tools/train_signs.py --data {args.output / 'data.yaml'}")


if __name__ == "__main__":
    main()
