"""Read-only census of what the lane tracker sees on a clip or recording.

Runs the real `LineTracker` and the real `Autonomous` controller over a video
for any operator Path and prints, per clip, what was seen and what would have
been commanded: which scan rows resolved, how many rows saw both lines, the
lane_state mix, offset / curvature / PID-error quantiles, every LOST and
provisional run, the controller's stop events, band visibility above the ROI,
and a timeline. Nothing here steers or writes to the tracker; it only reads.

Usage:
    .venv/bin/python tools/clip_census.py CLIP [CLIP ...]
        [--flip CLIP]...                 mirror that clip (detour_entry.mov was recorded mirrored)
        [--path normal|straight|detour]  anchor/profile for the whole clip (default normal)
        [--path detour@0,normal@12.5]    or a schedule: Detour from t=0 s, Normal from t=12.5 s
        [--fps F]                        override the container fps (time axis and controller dt)
        [--limit N]                      stop after N frames
        [--csv OUT.csv]                  per-frame rows (one file; clips are concatenated)
        [--rows]                         per-frame timeline instead of one line per 0.5 s
        [--roi X0,X1,Y0,Y1]              ROI fractions (defaults from Config)
        [--scan-lines N]                 scan rows (default from Config)

Examples:
    .venv/bin/python tools/clip_census.py clips/detour_exit.mov --path detour
    .venv/bin/python tools/clip_census.py clips/detour_entry.mov --flip clips/detour_entry.mov --path straight
    .venv/bin/python tools/clip_census.py assets/road.mov --limit 600 --csv /tmp/road.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import Counter
from dataclasses import dataclass, fields
from pathlib import Path as FsPath
from typing import Optional, Sequence

import cv2 as cv
import numpy as np

REPO_ROOT = FsPath(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from config import Config  # noqa: E402
from input.autonomous import Autonomous, _LOOKAHEAD_GAIN  # noqa: E402
from path import Path, anchor_for, profile_for  # noqa: E402
from perception import line_tracking  # noqa: E402
from perception.line_tracking import LineTracker  # noqa: E402

FALLBACK_FPS = 28.3
# |PID error| at which Kp alone saturates steering (1.0 / 0.0231 ~ 43 px).
FULL_LOCK_ERROR_PX = 43.0
TIMELINE_STEP_S = 0.5


# --- per-frame record ------------------------------------------------------


@dataclass
class FrameRecord:
    t: float
    frame: int
    path: str
    lane_state: str
    rows_found: int
    both_rows: int
    offset: Optional[int]
    curvature: Optional[float]
    pid_error: Optional[float]
    steering: float
    throttle: float
    holding: float
    profile: str
    provisional: bool
    band_rows: int
    # Not in the CSV: raw per-row data for the stats sections.
    row_indices: tuple[int, ...] = ()
    row_both: tuple[int, ...] = ()
    band_samples: tuple[tuple[int, int], ...] = ()
    stop_reason: Optional[str] = None
    profile_ready: bool = False


CSV_FIELDS = [
    "t", "frame", "path", "lane_state", "rows_found", "both_rows", "offset", "curvature",
    "pid_error", "steering", "throttle", "holding", "profile", "provisional", "band_rows",
]


# --- pure helpers ----------------------------------------------------------


def parse_path_schedule(spec: str) -> list[tuple[float, Path]]:
    """'detour@0,normal@12.5' -> [(0.0, DETOUR), (12.5, NORMAL)]; 'normal' -> [(0.0, NORMAL)]."""
    schedule = []
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        name, _, at = item.partition("@")
        try:
            path = Path(name.strip().lower())
        except ValueError:
            raise argparse.ArgumentTypeError(f"unknown path {name!r}; expected normal|straight|detour")
        schedule.append((float(at) if at else 0.0, path))
    if not schedule:
        raise argparse.ArgumentTypeError("empty path schedule")
    schedule.sort(key=lambda item: item[0])
    if schedule[0][0] > 0.0:
        schedule.insert(0, (0.0, schedule[0][1]))
    return schedule


def path_at(schedule: Sequence[tuple[float, Path]], t: float) -> Path:
    current = schedule[0][1]
    for at, path in schedule:
        if t >= at:
            current = path
    return current


def parse_roi(spec: str) -> tuple[float, float, float, float]:
    parts = [float(p) for p in spec.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("--roi wants X0,X1,Y0,Y1")
    return tuple(parts)  # type: ignore[return-value]


def pid_error(offset: Optional[int], curvature: Optional[float]) -> Optional[float]:
    if offset is None:
        return None
    return offset + _LOOKAHEAD_GAIN * (curvature or 0.0)


def find_runs(flags: Sequence[bool]) -> list[tuple[int, int]]:
    """[(start_index, length)] for every maximal run of True."""
    runs = []
    start = None
    for i, flag in enumerate(flags):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            runs.append((start, i - start))
            start = None
    if start is not None:
        runs.append((start, len(flags) - start))
    return runs


def quantiles(values: Sequence[float]) -> Optional[dict[str, float]]:
    if not values:
        return None
    arr = np.asarray(values, dtype=float)
    return {
        "p5": float(np.percentile(arr, 5)),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "min": float(arr.min()),
        "max": float(arr.max()),
    }


def state_token(lane_state: str) -> str:
    return lane_state.split(" ", 1)[0] if lane_state else "(empty)"


def transitions(values: Sequence[Optional[str]]) -> list[tuple[int, Optional[str], Optional[str]]]:
    """(index, from, to) every time the value changes."""
    out = []
    prev = None
    for i, value in enumerate(values):
        if value != prev:
            out.append((i, prev, value))
            prev = value
    return out


def band_slope(samples: Sequence[tuple[int, int]]) -> Optional[float]:
    if len({y for y, _ in samples}) < 2:
        return None
    return line_tracking._theil_sen(list(samples))[0]


def scan_rows(params: Config, frame_shape) -> tuple[list[int], int, int, int]:
    """The tracker's downscaled scan rows and (width, height, roi_y0) for its frame geometry."""
    ratio = params.downscale_ratio
    height = round(frame_shape[0] / ratio)
    width = round(frame_shape[1] / ratio)
    roi_y0 = round(height * max(0.0, min(params.roi_y0, params.roi_y1 - 0.02)))
    roi_y1 = round(height * min(1.0, params.roi_y1))
    ys = [int(y) for y in np.linspace(roi_y0, roi_y1, max(1, params.scan_lines), dtype=int)]
    return ys, width, height, roi_y0


def preprocess(frame: np.ndarray, params: Config) -> np.ndarray:
    """The tracker's binary image, byte for byte (see LineTracker.process)."""
    ratio = params.downscale_ratio
    height = round(frame.shape[0] / ratio)
    width = round(frame.shape[1] / ratio)
    small = cv.resize(frame, (width, height))
    gray = cv.cvtColor(small, cv.COLOR_BGR2GRAY)
    bk = max(3, params.blur_ksize | 1)
    blur = cv.medianBlur(gray, bk)
    bs = max(3, params.block_size | 1)
    return cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, bs, int(params.threshold_constant))


def band_row_ys(height: int, roi_y0: int) -> list[int]:
    return list(range(round(line_tracking._BAND_TOP * height), roi_y0, line_tracking._BAND_STEP))


# --- census ------------------------------------------------------------------


@dataclass
class Census:
    file: str
    size: tuple[int, int]
    fps: float
    fps_source: str
    flipped: bool
    schedule: list[tuple[float, Path]]
    params: Config
    records: list[FrameRecord]
    calibration_frame: Optional[int]
    width_profile: Optional[tuple[float, float]]
    clamped_frames: int
    band_ys: list[int]


def run_census(clip: str, schedule, params: Config, fps_override: Optional[float], flip: bool, limit: Optional[int]) -> Census:
    cap = cv.VideoCapture(clip)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {clip}")
    container_fps = cap.get(cv.CAP_PROP_FPS) or 0.0
    if fps_override:
        fps, fps_source = fps_override, "--fps"
    elif container_fps > 0:
        fps, fps_source = container_fps, "container"
    else:
        fps, fps_source = FALLBACK_FPS, "fallback"
    dt = 1.0 / fps

    tracker = LineTracker()
    controller = Autonomous()
    records: list[FrameRecord] = []
    calibration_frame = None
    size = (0, 0)
    scan_ys: list[int] = []
    band_ys: list[int] = []
    y_to_index: dict[int, int] = {}
    geometry = None

    index = 0
    while limit is None or index < limit:
        ok, frame = cap.read()
        if not ok:
            break
        if flip:
            frame = cv.flip(frame, 1)
        if geometry is None:
            size = (frame.shape[1], frame.shape[0])
            scan_ys, width, height, roi_y0 = scan_rows(params, frame.shape)
            geometry = (width, height, roi_y0)
            y_to_index = {y: i for i, y in enumerate(scan_ys)}
            band_ys = band_row_ys(height, roi_y0)
        width, height, roi_y0 = geometry

        t = index * dt
        path = path_at(schedule, t)
        # Band visibility from the same origin the tracker's scan uses this frame.
        scan_origin = tracker._prev_track_center if tracker._prev_track_center is not None else width // 2
        binary = preprocess(frame, params)
        scratch = np.zeros((height, width, 3), dtype=np.uint8)
        band = tracker._scan_band(binary, scratch, scan_origin, width, height, roi_y0)

        result = tracker.process(frame, params, anchor_for(path))
        if calibration_frame is None and result.profile_ready:
            calibration_frame = index

        steering, throttle = controller.step(
            result.lane_offset, result.lane_curvature, params, dt,
            profile=profile_for(path), provisional=result.provisional, rows_found=result.rows_found,
            far_offset=result.far_offset, far_curvature=result.far_curvature,
        )

        records.append(FrameRecord(
            t=t, frame=index, path=path.value, lane_state=result.lane_state,
            rows_found=result.rows_found, both_rows=result.both_rows,
            offset=result.lane_offset, curvature=result.lane_curvature,
            pid_error=pid_error(result.lane_offset, result.lane_curvature),
            steering=steering, throttle=throttle, holding=controller.holding,
            profile=controller.profile_label, provisional=result.provisional,
            band_rows=len(band),
            row_indices=tuple(y_to_index.get(y, -1) for y, _, _ in result.row_centers),
            row_both=tuple(y_to_index.get(y, -1) for y, _, both in result.row_centers if both),
            band_samples=tuple((int(y), int(w)) for y, w in band),
            stop_reason=controller.stop_reason, profile_ready=result.profile_ready,
        ))
        index += 1
    cap.release()

    return Census(
        file=clip, size=size, fps=fps, fps_source=fps_source, flipped=flip, schedule=schedule,
        params=params, records=records, calibration_frame=calibration_frame,
        width_profile=tracker._width_profile, clamped_frames=tracker.clamped_frames, band_ys=band_ys,
    )


# --- report ------------------------------------------------------------------


def fmt(value, digits=1, width=0) -> str:
    if value is None:
        s = "-"
    elif isinstance(value, float):
        s = f"{value:.{digits}f}"
    else:
        s = str(value)
    return s.rjust(width) if width else s


def pct(count: int, total: int) -> str:
    return f"{100.0 * count / total:5.1f}%" if total else "    -"


def print_histogram(title: str, counter: Counter, total: int, keys=None) -> None:
    print(f"\n{title}")
    keys = list(keys) if keys is not None else sorted(counter, key=lambda k: (-counter[k], str(k)))
    for key in keys:
        n = counter.get(key, 0)
        print(f"  {str(key):<16} {n:6d}  {pct(n, total)}")


def print_runs(title: str, runs, records, fps) -> None:
    print(f"\n{title}: {len(runs)}")
    if not runs:
        return
    print(f"  {'#':>3} {'start s':>8} {'end s':>8} {'frames':>7} {'seconds':>8}  first state -> last state")
    for i, (start, length) in enumerate(runs, 1):
        first, last = records[start], records[start + length - 1]
        print(f"  {i:>3} {first.t:8.2f} {last.t + 1 / fps:8.2f} {length:7d} {length / fps:8.2f}  {first.lane_state!r} -> {last.lane_state!r}")


def report(c: Census, rows: bool) -> None:
    recs = c.records
    n = len(recs)
    p = c.params
    print("=" * 78)
    print(f"file:        {c.file}")
    print(f"size:        {c.size[0]}x{c.size[1]}  fps: {c.fps:.3f} ({c.fps_source})  frames: {n}  duration: {n / c.fps:.2f} s  flipped: {c.flipped}")
    print(f"path:        {', '.join(f'{path.value}@{at:g}' for at, path in c.schedule)}")
    print(f"params:      roi=({p.roi_x0},{p.roi_x1},{p.roi_y0},{p.roi_y1}) scan_lines={p.scan_lines} downscale={p.downscale_ratio}")
    if c.calibration_frame is None:
        print(f"calibration: never ready ({line_tracking._CALIBRATION_FRAMES} frames needed)")
    else:
        print(f"calibration: ready at frame {c.calibration_frame} (t={c.calibration_frame / c.fps:.2f} s)")
    if c.width_profile is None:
        print("width profile: none")
    else:
        a, b = c.width_profile
        print(f"width profile: width = {a:.4f}*y + {b:.2f}  (clamped anchored frames: {c.clamped_frames})")
    if n == 0:
        print("no frames read")
        return

    # 2. per-row edge classes
    print(f"\nPer-row resolution (0 = far, {p.scan_lines - 1} = near)")
    print(f"  {'row':>3} {'resolved':>9} {'both':>9}")
    resolved = Counter(i for r in recs for i in r.row_indices)
    both = Counter(i for r in recs for i in r.row_both)
    for i in range(p.scan_lines):
        print(f"  {i:>3} {pct(resolved[i], n):>9} {pct(both[i], n):>9}")

    # 3. both_rows / rows_found histograms
    print_histogram("both_rows histogram", Counter(r.both_rows for r in recs), n, range(p.scan_lines + 1))
    print_histogram("rows_found histogram", Counter(r.rows_found for r in recs), n, range(p.scan_lines + 1))

    # 4. lane_state histogram
    print_histogram("lane_state histogram (first token)", Counter(state_token(r.lane_state) for r in recs), n)
    full_states = Counter(r.lane_state for r in recs)
    print("  most common full labels:")
    for label, count in full_states.most_common(6):
        print(f"    {label!r:<40} {count:6d}  {pct(count, n)}")

    # 5. quantiles
    print("\nQuantiles (frames with an offset)")
    print(f"  {'':<10} {'p5':>8} {'p50':>8} {'p95':>8} {'min':>8} {'max':>8}")
    series = {
        "offset": [r.offset for r in recs if r.offset is not None],
        "curvature": [r.curvature for r in recs if r.curvature is not None],
        "pid_error": [r.pid_error for r in recs if r.pid_error is not None],
    }
    for name, values in series.items():
        q = quantiles(values)
        if q is None:
            print(f"  {name:<10} (no values)")
            continue
        d = 3 if name == "curvature" else 1
        print(f"  {name:<10} " + " ".join(fmt(q[k], d, 8) for k in ("p5", "p50", "p95", "min", "max")) + f"   n={len(values)}")
    errors = series["pid_error"]
    saturated = sum(1 for e in errors if abs(e) >= FULL_LOCK_ERROR_PX)
    print(f"  |pid_error| >= {FULL_LOCK_ERROR_PX:g} px (full lock at Kp {p.kp}): {saturated}/{len(errors)} = {pct(saturated, len(errors))} of offset frames, {pct(saturated, n)} of all frames")

    # 6. LOST runs, provisional runs, stop events
    print_runs("LOST runs (lane_offset is None)", find_runs([r.offset is None for r in recs]), recs, c.fps)
    print_runs("Provisional runs", find_runs([r.provisional for r in recs]), recs, c.fps)
    stops = [(i, prev, new) for i, prev, new in transitions([r.stop_reason for r in recs]) if i > 0 or new is not None]
    print(f"\nController stop events (stop_reason transitions): {len(stops)}")
    for i, prev, new in stops:
        print(f"  t={recs[i].t:7.2f} s  frame {i:5d}  {prev!r} -> {new!r}  profile={recs[i].profile}")
    max_hold = max(recs, key=lambda r: r.holding)
    print(f"max holding: {max_hold.holding:.2f} s (t={max_hold.t:.2f} s, frame {max_hold.frame})")
    stopped = sum(1 for r in recs if r.stop_reason is not None)
    print(f"frames with a stop_reason: {stopped}  {pct(stopped, n)}")

    # 7. band visibility
    print(f"\nBand visibility (rows y={c.band_ys[0] if c.band_ys else '-'}..{c.band_ys[-1] if c.band_ys else '-'} step {line_tracking._BAND_STEP}, {len(c.band_ys)} rows, downscaled px)")
    print_histogram("band rows with both lines per frame", Counter(r.band_rows for r in recs), n, range(len(c.band_ys) + 1))
    two_plus = sum(1 for r in recs if r.band_rows >= 2)
    print(f"  frames with >= 2 band rows: {two_plus}  {pct(two_plus, n)}")
    if c.band_ys:
        top_y, bottom_y = c.band_ys[0], c.band_ys[-1]
        top_w = [w for r in recs for y, w in r.band_samples if y == top_y]
        bottom_w = [w for r in recs for y, w in r.band_samples if y == bottom_y]
        print(f"  median band width at top row y={top_y}: {fmt(float(np.median(top_w)) if top_w else None)} px (n={len(top_w)})")
        print(f"  median band width at bottom row y={bottom_y}: {fmt(float(np.median(bottom_w)) if bottom_w else None)} px (n={len(bottom_w)})")
    slopes = [s for s in (band_slope(r.band_samples) for r in recs) if s is not None]
    print(f"  width-vs-row slope, median of per-frame Theil-Sen: {fmt(float(np.median(slopes)) if slopes else None, 4)} px/row (n={len(slopes)})")
    pooled = [s for r in recs for s in r.band_samples]
    print(f"  width-vs-row slope, pooled least squares: {fmt(float(np.polyfit([y for y, _ in pooled], [w for _, w in pooled], 1)[0]) if len({y for y, _ in pooled}) >= 2 else None, 4)} px/row")

    # 8. timeline
    print("\nTimeline" + (" (every frame)" if rows else f" (every {TIMELINE_STEP_S:g} s)"))
    print(f"  {'t':>6} {'fr':>5} {'path':<8} {'lane_state':<36} {'rf':>2} {'br':>2} {'off':>5} {'curv':>7} {'err':>7} {'steer':>6} {'thr':>5} {'hold':>5} profile")
    step = 1 if rows else max(1, round(TIMELINE_STEP_S * c.fps))
    for r in recs[::step]:
        print(f"  {r.t:6.2f} {r.frame:5d} {r.path:<8} {r.lane_state:<36} {r.rows_found:>2} {r.both_rows:>2} "
              f"{fmt(r.offset, 0, 5)} {fmt(r.curvature, 3, 7)} {fmt(r.pid_error, 1, 7)} {r.steering:6.2f} {r.throttle:5.2f} {r.holding:5.2f} {r.profile}"
              + ("  PROV" if r.provisional else "") + (f"  stop={r.stop_reason}" if r.stop_reason else ""))


def write_csv(path: str, censuses: Sequence[Census]) -> None:
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["clip"] + CSV_FIELDS)
        for c in censuses:
            for r in c.records:
                writer.writerow([c.file] + [getattr(r, f) for f in CSV_FIELDS])


# --- main --------------------------------------------------------------------


def same_file(a: str, b: str) -> bool:
    try:
        return os.path.samefile(a, b)
    except OSError:
        return os.path.abspath(a) == os.path.abspath(b)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("clips", nargs="+")
    ap.add_argument("--flip", action="append", default=[], metavar="CLIP", help="mirror this clip horizontally")
    ap.add_argument("--path", type=parse_path_schedule, default=parse_path_schedule("normal"), help="normal|straight|detour or a schedule like detour@0,normal@12.5")
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--rows", action="store_true", help="per-frame timeline")
    ap.add_argument("--roi", type=parse_roi, default=None, metavar="X0,X1,Y0,Y1")
    ap.add_argument("--scan-lines", type=int, default=None)
    args = ap.parse_args(argv)

    params = Config()
    if args.roi:
        params.roi_x0, params.roi_x1, params.roi_y0, params.roi_y1 = args.roi
    if args.scan_lines:
        params.scan_lines = args.scan_lines

    censuses = []
    for clip in args.clips:
        flip = any(same_file(clip, f) for f in args.flip)
        census = run_census(clip, args.path, params, args.fps, flip, args.limit)
        report(census, args.rows)
        censuses.append(census)
    if args.csv:
        write_csv(args.csv, censuses)
        print(f"\nwrote {sum(len(c.records) for c in censuses)} rows to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
