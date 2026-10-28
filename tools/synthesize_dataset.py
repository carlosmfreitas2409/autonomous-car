#!/usr/bin/env python3


import argparse
import json
import random
import shutil
from pathlib import Path

import cv2 as cv
import numpy as np

CLASSES = ("delivery_sign", "detour", "stop", "traffic_light")

FRAME_SIZE = (1280, 720)

PASTE_HEIGHT = {
    "traffic_light": (22, 640),
    "delivery_sign": (22, 560),
    "detour": (22, 560),
    "stop": (22, 560),
}

NO_SIGN_SHARE = 1 / 6
DISTRACTOR_SHARE = 0.5
VAL_SHARE = 0.15

REAL_SHARE = 0.2

PATCH_PASTE_SHARE = 0.5

DARKEN = (0.30, 1.05)

FIXTURE_SHARE = 0.5
FIXTURE_HEIGHT = (12, 160)

LIT_COLOUR = {"red": (55, 60, 235), "yellow": (60, 215, 240), "green": (95, 220, 105)}

GLOW_REACH = (1.1, 3.0)
GLOW_STRENGTH = (0.25, 0.75)

GLOW_VEIL = (0.05, 0.30)


def warp_cutout(bgra: np.ndarray, rng: random.Random, extra: np.ndarray = None):
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
    out[:, :, 3] = np.clip(alpha * 255 + halo * 200 * strength, 0, 255).astype(np.uint8)
    return out


def lit_lamp(name: str):
    head = name.split("_")[0].lower()
    if head not in LIT_COLOUR:
        return None
    return {"red": 0, "yellow": 1, "green": 2}[head], LIT_COLOUR[head]


def scene_glow(frame: np.ndarray, box, cell: int, colour, rng: random.Random) -> None:
    x0, y0, x1, y1 = box
    height = max(1, y1 - y0)
    centre_x = (x0 + x1) / 2.0
    centre_y = y0 + height * (0.18, 0.5, 0.82)[cell]

    reach = height * rng.uniform(*GLOW_REACH)
    strength = rng.uniform(*GLOW_STRENGTH)
    tint = np.float32(colour)[None, None, :]

    frame_h, frame_w = frame.shape[:2]
    left, right = int(max(0, centre_x - reach)), int(min(frame_w, centre_x + reach))
    top, bottom = int(max(0, centre_y - reach)), int(min(frame_h, centre_y + reach))
    if right - left >= 4 and bottom - top >= 4:
        yy, xx = np.mgrid[top:bottom, left:right]
        distance = np.hypot(xx - centre_x, yy - centre_y) / reach
        falloff = np.clip(1.0 - distance ** 2, 0.0, 1.0) ** rng.uniform(1.2, 2.2)
        falloff = (falloff * strength)[:, :, None]
        region = frame[top:bottom, left:right].astype(np.float32)
        frame[top:bottom, left:right] = np.clip(
            region * (1 - falloff) + np.maximum(region, tint) * falloff, 0, 255).astype(np.uint8)

    fill = np.clip(height / frame_h, 0.0, 1.0)
    veil = rng.uniform(*GLOW_VEIL) * fill
    if veil > 0.01:
        whole = frame.astype(np.float32)
        frame[:] = np.clip(whole * (1 - veil) + tint * veil, 0, 255).astype(np.uint8)


def render_onto(frame: np.ndarray, bgra: np.ndarray, height: int,
                rng: random.Random, position=None, box_alpha: np.ndarray = None):

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


def degrade_frame(frame: np.ndarray, rng: random.Random,
                  gentle: bool = False) -> np.ndarray:

    out = frame.astype(np.float32)

    gain = rng.uniform(0.85, 1.15) if gentle else rng.uniform(0.55, 1.45)
    gamma = rng.uniform(0.92, 1.10) if gentle else rng.uniform(0.75, 1.35)
    out = 255.0 * np.power(np.clip(out * gain / 255.0, 0, 1), gamma)

    spread = (0.97, 1.04) if gentle else (0.9, 1.12)
    cast = np.float32([rng.uniform(*spread) for _ in range(3)])
    out = np.clip(out * cast, 0, 255)

    choice = 1.0 if gentle and rng.random() < 0.6 else rng.random()
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

    out += np.random.normal(0, rng.uniform(1.0, 4.0) if gentle else rng.uniform(1.5, 9.0),
                            out.shape)
    out = np.clip(out, 0, 255).astype(np.uint8)

    quality = rng.randint(62, 95) if gentle else rng.randint(38, 92)
    ok, buffer = cv.imencode(".jpg", out, [cv.IMWRITE_JPEG_QUALITY, quality])
    return cv.imdecode(buffer, cv.IMREAD_COLOR) if ok else out


def draw_distractors(frame: np.ndarray, rng: random.Random, fixtures=()) -> None:
    frame_h, frame_w = frame.shape[:2]
    if fixtures and rng.random() < FIXTURE_SHARE:
        for _ in range(rng.randint(1, 3)):
            fixture = rng.choice(fixtures).copy()
            fixture[:, :, :3] = np.clip(fixture[:, :, :3] * rng.uniform(*DARKEN), 0, 255)
            if rng.random() < 0.5:
                fixture = cv.flip(fixture, 1)
            height = int(round(np.exp(rng.uniform(*np.log(FIXTURE_HEIGHT)))))
            render_onto(frame, fixture, height, rng)
    for _ in range(rng.randint(1, 4)):
        kind = rng.random()
        colour = rng.choice([(40, 40, 255), (40, 220, 255), (60, 230, 90),
                             (255, 255, 255), (255, 190, 120)])
        if kind < 0.45:
            radius = rng.randint(6, 90)
            cx, cy = rng.randint(0, frame_w), rng.randint(0, frame_h)
            count = rng.choice((1, 1, 2))
            step = rng.randint(2, 4) * radius
            for index in range(count):
                _glowing_disc(frame, cx, cy + index * step, radius, colour, rng)
        elif kind < 0.7:
            width = rng.randint(20, 240)
            height = max(3, int(width * rng.uniform(0.04, 0.25)))
            x, y = rng.randint(0, frame_w), rng.randint(0, frame_h)
            overlay = frame.copy()
            cv.rectangle(overlay, (x, y), (x + width, y + height), colour, -1)
            cv.addWeighted(overlay, rng.uniform(0.5, 1.0), frame, rng.uniform(0.0, 0.5), 0, frame)
        else:
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


def load_clip_samples(directory: Path) -> list:
    samples = []
    for manifest in sorted(directory.glob("*.json")):
        spec = json.loads(manifest.read_text())
        clip = Path(spec["clip"])
        if not clip.exists():
            print(f"  {manifest.name}: {clip} is missing, skipping")
            continue
        wanted = {int(k): v for k, v in spec["boxes"].items()}
        capture = cv.VideoCapture(str(clip))
        index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if index in wanted:
                samples.append((frame, [(spec["label"], tuple(wanted[index]))]))
            index += 1
        capture.release()
        print(f"  {manifest.name}: {len(wanted)} verified frames from {clip.name}")
    return samples


def load_labelled_folders(root: Path) -> list:
    images_root, labels_root = root / "images", root / "labels"
    if not images_root.exists():
        return []
    samples = []
    for image_path in sorted(images_root.rglob("*")):
        if image_path.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        label_path = labels_root / image_path.relative_to(images_root).with_suffix(".txt")
        if not label_path.exists():
            continue
        frame = cv.imread(str(image_path))
        if frame is None:
            continue
        height, width = frame.shape[:2]
        boxes = []
        for line in label_path.read_text().splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            cx, cy, w, h = (float(v) for v in parts[1:])
            boxes.append((CLASSES[int(parts[0])],
                          ((cx - w / 2) * width, (cy - h / 2) * height,
                           (cx + w / 2) * width, (cy + h / 2) * height)))
        samples.append((frame, boxes))
    return samples


def real_sample(frame, boxes, rng: random.Random):
    if not boxes:
        return None
    frame_h, frame_w = frame.shape[:2]
    _, anchor = boxes[rng.randrange(len(boxes))]
    x0, y0, x1, y1 = anchor
    width, height = x1 - x0, y1 - y0

    margin = rng.uniform(0.15, 3.0)
    wx0 = rng.uniform(max(0, x0 - width * margin), x0)
    wy0 = rng.uniform(max(0, y0 - height * margin), y0)
    wx1 = rng.uniform(x1, min(frame_w, x1 + width * margin))
    wy1 = rng.uniform(y1, min(frame_h, y1 + height * margin))
    if wx1 - wx0 < 32 or wy1 - wy0 < 32:
        return None

    aspect = FRAME_SIZE[0] / FRAME_SIZE[1]
    centre_x, centre_y = (wx0 + wx1) / 2, (wy0 + wy1) / 2
    half_w, half_h = (wx1 - wx0) / 2, (wy1 - wy0) / 2
    if half_w / half_h < aspect:
        half_w = half_h * aspect
    else:
        half_h = half_w / aspect
    half_w = min(half_w, frame_w / 2, (frame_h / 2) * aspect)
    half_h = half_w / aspect
    centre_x = float(np.clip(centre_x, half_w, frame_w - half_w))
    centre_y = float(np.clip(centre_y, half_h, frame_h - half_h))
    wx0, wx1 = centre_x - half_w, centre_x + half_w
    wy0, wy1 = centre_y - half_h, centre_y + half_h

    window = frame[int(wy0):int(wy1), int(wx0):int(wx1)]
    if window.shape[0] < 16 or window.shape[1] < 16:
        return None
    scale_x = FRAME_SIZE[0] / window.shape[1]
    scale_y = FRAME_SIZE[1] / window.shape[0]
    out = cv.resize(window, FRAME_SIZE)
    flip = rng.random() < 0.5
    if flip:
        out = cv.flip(out, 1)

    kept = []
    for label, (bx0, by0, bx1, by1) in boxes:
        inside_w = min(bx1, wx1) - max(bx0, wx0)
        inside_h = min(by1, wy1) - max(by0, wy0)
        if inside_w <= 0 or inside_h <= 0:
            continue
        if inside_w * inside_h < 0.92 * (bx1 - bx0) * (by1 - by0):
            continue
        moved = ((bx0 - wx0) * scale_x, (by0 - wy0) * scale_y,
                 (bx1 - wx0) * scale_x, (by1 - wy0) * scale_y)
        if flip:
            moved = (FRAME_SIZE[0] - moved[2], moved[1], FRAME_SIZE[0] - moved[0], moved[3])
        if moved[2] - moved[0] < 6 or moved[3] - moved[1] < 6:
            continue
        kept.append((label, tuple(int(v) for v in moved)))
    if not kept:
        return None
    return out, kept


def real_patch_paste(samples: list, backgrounds: list, rng: random.Random):
    for _ in range(12):
        frame, boxes = samples[rng.randrange(len(samples))]
        if not boxes:
            continue
        label, (x0, y0, x1, y1) = boxes[rng.randrange(len(boxes))]
        width, height = x1 - x0, y1 - y0
        if width < 24 or height < 24:
            continue

        reach = rng.uniform(1.5, 2.6)
        px0, py0 = x0 - width * reach, y0 - height * reach
        px1, py1 = x1 + width * reach, y1 + height * reach
        frame_h, frame_w = frame.shape[:2]
        if px0 < 0 or py0 < 0 or px1 > frame_w or py1 > frame_h:
            continue
        if any(other is not boxes[0] and not (b[2] <= px0 or b[0] >= px1 or b[3] <= py0 or b[1] >= py1)
               for other, b in ((o, o[1]) for o in boxes if o[1] != (x0, y0, x1, y1))):
            continue

        patch = frame[int(py0):int(py1), int(px0):int(px1)]
        if patch.size == 0:
            continue

        target = np.exp(rng.uniform(np.log(34), np.log(200)))
        scale = target / height
        new_w = max(8, int(round(patch.shape[1] * scale)))
        new_h = max(8, int(round(patch.shape[0] * scale)))
        if new_w >= FRAME_SIZE[0] or new_h >= FRAME_SIZE[1]:
            continue
        patch = cv.resize(patch, (new_w, new_h), interpolation=cv.INTER_AREA)

        canvas = background_frame(rng.choice(backgrounds), rng)
        at_x = rng.randint(0, FRAME_SIZE[0] - new_w)
        at_y = rng.randint(0, FRAME_SIZE[1] - new_h)

        region = canvas[at_y:at_y + new_h, at_x:at_x + new_w].astype(np.float32)
        patch = patch.astype(np.float32)

        rim = max(1, int(0.12 * min(new_h, new_w)))
        patch_rim = float(np.median(np.concatenate([patch[:rim].ravel(), patch[-rim:].ravel()])))
        scene_rim = float(np.median(np.concatenate([region[:rim].ravel(), region[-rim:].ravel()])))
        patch = np.clip(patch * float(np.clip((scene_rim + 12) / (patch_rim + 12), 0.45, 2.2)), 0, 255)

        yy, xx = np.mgrid[0:new_h, 0:new_w]
        radius = np.hypot((xx - new_w / 2) / (new_w / 2), (yy - new_h / 2) / (new_h / 2))
        alpha = np.clip(1.0 - (radius - 0.45) / 0.55, 0.0, 1.0) ** 1.5
        alpha = alpha[:, :, None]

        canvas[at_y:at_y + new_h, at_x:at_x + new_w] = (
            region * (1 - alpha) + patch * alpha).astype(np.uint8)

        moved = (at_x + (x0 - px0) * scale, at_y + (y0 - py0) * scale,
                 at_x + (x1 - px0) * scale, at_y + (y1 - py0) * scale)
        return canvas, [(label, tuple(int(v) for v in moved))]
    return None


def clip_negative_crops(samples: list, rng: random.Random, count: int) -> list:
    crops = []
    if not samples:
        return crops
    attempts = 0
    while len(crops) < count and attempts < count * 20:
        attempts += 1
        frame, boxes = samples[rng.randrange(len(samples))]
        if not boxes:
            continue
        x0 = min(b[1][0] for b in boxes); x1 = max(b[1][2] for b in boxes)
        frame_h, frame_w = frame.shape[:2]
        gaps = [("left", 0, x0), ("right", x1, frame_w)]
        side, low, high = max(gaps, key=lambda g: g[2] - g[1])
        if high - low < 120:
            continue
        cx0 = rng.randrange(int(low), int(high) - 100)
        cx1 = rng.randrange(cx0 + 100, int(high) + 1)
        cy0 = rng.randrange(0, max(1, frame_h - 100))
        cy1 = rng.randrange(cy0 + 100, frame_h + 1)
        crops.append(cv.resize(frame[cy0:cy1, cx0:cx1], FRAME_SIZE))
    return crops


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


def load_fixtures(root: Path) -> list:
    fixtures = []
    for path in sorted(root.glob("*.png")):
        image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
        if image is None or image.shape[2] != 4:
            print(f"  skipping {path}: needs an alpha channel")
            continue
        fixtures.append(image)
    return fixtures


def load_backgrounds(directories) -> list:
    paths = []
    for directory in directories:
        directory = Path(directory)
        if not directory.exists():
            continue
        paths += [p for p in sorted(directory.rglob("*"))
                  if p.suffix.lower() in (".jpg", ".jpeg", ".png")]
    return paths


def background_frame(path: Path, rng: random.Random, darken: bool = True) -> np.ndarray:
    image = cv.imread(str(path))
    if image is None:
        return np.full((FRAME_SIZE[1], FRAME_SIZE[0], 3), 40, np.uint8)
    height, width = image.shape[:2]
    scale = rng.uniform(0.6, 1.0)
    crop_w, crop_h = int(width * scale), int(height * scale)
    x = rng.randint(0, width - crop_w)
    y = rng.randint(0, height - crop_h)
    cropped = image[y:y + crop_h, x:x + crop_w]
    frame = cv.resize(cropped, FRAME_SIZE)
    if rng.random() < 0.5:
        frame = cv.flip(frame, 1)
    if darken:
        frame = np.clip(frame.astype(np.float32) * rng.uniform(*DARKEN), 0, 255).astype(np.uint8)
    return frame


def make_sample(frame, cutouts, rng):
    if rng.random() < NO_SIGN_SHARE:
        return None
    label = rng.choice([c for c in CLASSES if cutouts[c]])
    name, cutout = rng.choice(cutouts[label])
    low, high = PASTE_HEIGHT[label]
    height = int(round(np.exp(rng.uniform(np.log(low), np.log(high)))))
    housing_alpha = cutout[:, :, 3].copy()
    lamp = lit_lamp(name) if label == "traffic_light" else None
    if label == "traffic_light" and rng.random() < 0.7:
        cutout = bloom(cutout, rng)
    box = render_onto(frame, cutout, height, rng, box_alpha=housing_alpha)
    if box is None:
        return None
    if lamp is not None and rng.random() < 0.85:
        scene_glow(frame, box, lamp[0], lamp[1], rng)
    return [(label, box)]


def yolo_line(label: str, box, shape) -> str:
    height, width = shape[:2]
    x0, y0, x1, y1 = box
    cx, cy = (x0 + x1) / 2 / width, (y0 + y1) / 2 / height
    return f"{CLASSES.index(label)} {cx:.6f} {cy:.6f} {(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=4000)
    parser.add_argument("--out", default="dataset/synth")
    parser.add_argument("--cutouts", default="assets/sign_cutouts")
    parser.add_argument("--backgrounds", nargs="+", default=["dataset/backgrounds"])
    parser.add_argument("--distractors", default="assets/distractor_cutouts",
                        help="real non-sign objects pasted unlabelled; '' for none")
    parser.add_argument("--real-images", default="assets/real_train",
                        help="hand-labelled real stills to train on; '' for none")
    parser.add_argument("--clip-labels", default="assets/clip_labels",
                        help="hand-verified boxes for real footage; '' to train synthetic only")
    parser.add_argument("--real-share", type=float, default=REAL_SHARE,
                        help="fraction of output frames taken from that real footage")
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
    fixtures = load_fixtures(Path(args.distractors)) if args.distractors else []
    print(f"{sum(len(v) for v in cutouts.values())} cutouts, {len(backgrounds)} backgrounds, "
          f"{len(fixtures)} fixture distractors")

    clip_samples = []
    if args.clip_labels and Path(args.clip_labels).exists():
        clip_samples = load_clip_samples(Path(args.clip_labels))
    if args.real_images and Path(args.real_images).exists():
        folders = load_labelled_folders(Path(args.real_images))
        objects = sum(len(b) for _, b in folders)
        print(f"  {args.real_images}: {len(folders)} labelled stills, {objects} objects")
        clip_samples += folders
    if not clip_samples:
        print("  no real footage - training on composites alone, which is what")
        print("  scored 0.000 on every lit venue photo last time")
    negatives = clip_negative_crops(clip_samples, rng, count=min(120, 3 * len(clip_samples)))
    print(f"{len(clip_samples)} real frames, {len(negatives)} real negative crops")

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
    real_used = 0
    for index in range(args.count):
        split = "val" if rng.random() < VAL_SHARE else "train"
        sample = None
        from_real = False
        if clip_samples and rng.random() < args.real_share:
            made = (real_patch_paste(clip_samples, backgrounds, rng)
                    if rng.random() < PATCH_PASTE_SHARE else None)
            if made is None:
                real, boxes = clip_samples[rng.randrange(len(clip_samples))]
                made = real_sample(real, boxes, rng)
            if made is not None:
                frame, kept = made
                sample = kept
                from_real = True
                real_used += 1
            else:
                frame = background_frame(rng.choice(backgrounds), rng)
        elif negatives and rng.random() < 0.12:
            frame = negatives[rng.randrange(len(negatives))].copy()
        else:
            frame = background_frame(rng.choice(backgrounds), rng)
            if rng.random() < DISTRACTOR_SHARE:
                draw_distractors(frame, rng, fixtures)
            sample = make_sample(frame, cutouts, rng)
        frame = degrade_frame(frame, rng, gentle=from_real)

        lines = []
        if sample:
            for label, box in sample:
                lines.append(yolo_line(label, box, frame.shape))
                counts[label] += 1
        else:
            empty += 1

        name = f"{index:06d}"
        cv.imwrite(str(out / "images" / split / f"{name}.jpg"), frame)
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
    print(f"  from real footage: {real_used}")
    if args.preview:
        print(f"\nLook at {preview_dir} before training. If the boxes are not tight")
        print("on the signs, nothing downstream matters.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
