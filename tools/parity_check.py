#!/usr/bin/env python3
"""Prove the working path is untouched and exercise the detour paths, offline.

The detour work only adds paths: Normal must come out of it bit for bit the
same as the pinned baseline (`--ref`, default `bb5853e`). What matters is not
`lane_offset` on its own but the number the PID actually sees, so the tracker
is compared on `lane_offset + _LOOKAHEAD_GAIN * lane_curvature` frame by frame
with every live-tunable slider re-randomised every 30 frames from a fixed
seed; the controller is compared on `(steering, throttle)` every step against
the baseline `Autonomous` fed the same random sequences.

Checks, in order:

1. AUTO parity (wide and tuned sliders, default-argument and explicit-AUTO
   calls alternating): identical PID error on every frame of every clip.
2. Controller parity: NORMAL profile identical to the baseline on random
   offset/curvature sequences with None runs, random dt, resets, with the
   throttle scaled by a random sonar-guard scale (floored at the crawl
   throttle, so the car reaches the stop instead of stalling short of it). The sonar guard itself
   (`src/obstacle.py`) is checked on scripted scenes: the old ramp dead ahead,
   side sensors on a shorter ramp, stop latch hysteresis, one dropped or
   ghost echo ignored, stale data stops; and an obstacle stop holds the wheels.
3. Calibration: at `Config()` defaults the width profile is ready within 60
   frames (or 2 s) of every clip (the ROI alone never has two both-lines rows on this
   camera; the band above it does).
4. Learning guard: a Normal pass over the entry clip; the profile slope
   stays within 5 % from the moment the fork opens until both lines are seen
   again (the fork must not teach).
5. Operator replay: scripted Normal -> Detour -> home on each clip, plus
   "forgot L1" and "early L1", through the real `Autonomous` at the clip's
   fps: no LOST run longer than the hold, no provisional frame ends a Detour
   hold, the slew respected, a hold never shortened by a path change, and
   Normal after Detour re-converging to a fresh Normal tracker.
5b. Automatic return: armed at the sign on the day-1 recording and the
   entry clip, the car goes home after the entrance gap and before the merge,
   and the carried hold takes it through the merge without a stop.
6. Bias: Straight (LEFT) against Normal on the frames where Normal saw two
   both-lines rows (mean PID diff <= 10 px, curvature <= 0.1). Passing this on
   the day-0 Normal-lap recording is the entry condition for driving Straight.
7. Smoke: LEFT and RIGHT held on all footage; nothing raised from the
   anchored scan, the profile finite after every wide-slider pass.

Extreme slider combinations can make the AUTO scan index outside the frame (a
previous centre learned from one edge can sit beyond the ROI). That predates
this change and is not fixed here: check 1 requires both trackers to fail the
same way on the same frame; the smoke check only accepts such a failure when
it comes from the AUTO fallback rather than from the anchored scan.

    python tools/parity_check.py assets/road.mov clips/detour_exit.mov \
        clips/detour_entry.mov --flip clips/detour_entry.mov
"""

import argparse
import importlib.util
import math
import random
import subprocess
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass, replace
from pathlib import Path as FilePath
from typing import Optional

REPO_ROOT = FilePath(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

import cv2 as cv

from config import Config
from input import autonomous as autonomous_module
from input.autonomous import _LOOKAHEAD_GAIN, Autonomous, ControlProfile
import obstacle as obstacle_module
from obstacle import ObstacleGuard
from path import HOME_PATH, DetourReturn, Path, anchor_for, profile_for
from perception.line_tracking import LaneAnchor, LineTracker

# Re-randomise the sliders this often, so a run covers many parameter sets
# without hiding a divergence that only shows up after some frames of state.
_PARAM_PERIOD = 30
_SEED = 20260903
_DEFAULT_REF = "bb5853e"
_CALIBRATION_BUDGET_FRAMES = 60
_LEARNING_GUARD_TOL = 0.05
_FORK_GAP_FRAMES = 10
_RECONVERGE_FRAMES = 30
_RECONVERGE_TOL_PX = 5.0


def load_baseline(ref: str, relpath: str, name: str):
    """Import a module as it exists on `ref`, alongside the current one."""
    source = subprocess.run(
        ["git", "show", f"{ref}:{relpath}"],
        cwd=REPO_ROOT, capture_output=True, text=True, check=True,
    ).stdout

    path = FilePath(tempfile.mkdtemp()) / f"{name}.py"
    path.write_text(source)

    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def random_params(rng: random.Random) -> Config:
    """A Config drawn from the GUI slider ranges (perception fields only)."""
    return Config(
        block_size=rng.randrange(3, 152, 2),
        threshold_constant=rng.randint(-50, 50),
        blur_ksize=rng.randrange(3, 16, 2),
        roi_x0=round(rng.uniform(0.0, 0.49), 2),
        roi_x1=round(rng.uniform(0.51, 1.0), 2),
        roi_y0=round(rng.uniform(0.1, 0.95), 2),
        roi_y1=round(rng.uniform(0.5, 0.99), 2),
        ema_alpha=round(rng.uniform(0.01, 1.0), 2),
        scan_lines=rng.randint(1, 15),
    )


def tuned_params(rng: random.Random) -> Config:
    """The narrower ranges: sliders somebody driving the car would settle on."""
    return Config(
        block_size=rng.randrange(51, 152, 2),
        threshold_constant=rng.randint(-40, -10),
        blur_ksize=rng.randrange(3, 16, 2),
        roi_x0=round(rng.uniform(0.0, 0.15), 2),
        roi_x1=round(rng.uniform(0.85, 1.0), 2),
        roi_y0=round(rng.uniform(0.5, 0.8), 2),
        roi_y1=round(rng.uniform(0.85, 0.99), 2),
        ema_alpha=round(rng.uniform(0.1, 0.8), 2),
        scan_lines=rng.randint(3, 12),
    )


def pid_error(result):
    """The value `Autonomous.step` feeds the PID, or None when the lane is lost."""
    if result.lane_offset is None:
        return None
    curvature = result.lane_curvature if result.lane_curvature is not None else 0.0
    return result.lane_offset + _LOOKAHEAD_GAIN * curvature


def outcome(tracker, frame, params, *args):
    """(pid error, exception) - a raise is an outcome to compare, not a stop."""
    try:
        return pid_error(tracker.process(frame, params, *args)), None
    except Exception as error:  # noqa: BLE001 - the point is to compare it
        return None, error


def raised_in(error: Exception, name: str) -> bool:
    tb = error.__traceback__
    while tb is not None:
        if tb.tb_frame.f_code.co_name == name:
            return True
        tb = tb.tb_next
    return False


@dataclass(frozen=True)
class Clip:
    path: str
    flip: bool = False

    @property
    def name(self) -> str:
        return FilePath(self.path).name + (" (flipped)" if self.flip else "")

    def fps(self) -> float:
        cap = cv.VideoCapture(self.path)
        fps = cap.get(cv.CAP_PROP_FPS)
        cap.release()
        return fps if fps and fps == fps and fps > 0 else 28.3


def frames(clip: Clip, limit=None):
    cap = cv.VideoCapture(clip.path)
    if not cap.isOpened():
        sys.exit(f"Could not open {clip.path}")
    index = 0
    while limit is None or index < limit:
        ok, frame = cap.read()
        if not ok:
            break
        if clip.flip:
            frame = cv.flip(frame, 1)
        yield index, frame
        index += 1
    cap.release()


def replay(clip: Clip, limit=None, maker=random_params):
    """(index, frame, params) with the sliders re-randomised periodically."""
    rng = random.Random(_SEED)
    params = maker(rng)
    for index, frame in frames(clip, limit):
        if index % _PARAM_PERIOD == 0:
            params = maker(rng)
        yield index, frame, params


def mean(values) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


# --- 1. AUTO parity -------------------------------------------------------

def run_auto_parity(clip: Clip, baseline_module, limit, maker=random_params) -> bool:
    baseline = baseline_module.LineTracker()
    current = LineTracker()
    mismatches = []
    total = 0
    shared_failures = 0

    for index, frame, params in replay(clip, limit, maker):
        want, want_error = outcome(baseline, frame, params)
        # Alternate the default argument and an explicit AUTO: both are Normal.
        got, got_error = outcome(current, frame, params) if index % 2 else outcome(current, frame, params, LaneAnchor.AUTO)
        total += 1

        if want_error is not None or got_error is not None:
            if repr(want_error) != repr(got_error):
                mismatches.append((index, want_error, got_error))
            else:
                # Same pre-existing failure on both sides. Reset together so the
                # replay keeps running from a matching state.
                shared_failures += 1
                baseline.reset()
                current.reset()
            continue

        if want != got:
            mismatches.append((index, want, got))

    sliders = "wide sliders" if maker is random_params else "tuned sliders"
    print(f"[{clip.name}] AUTO parity, {sliders}: {total - len(mismatches)}/{total} frames identical"
          f" ({shared_failures} of them the same pre-existing out-of-frame scan on both sides)")
    for index, want, got in mismatches[:5]:
        print(f"  frame {index}: baseline {want!r} != current {got!r}")
    if len(mismatches) > 5:
        print(f"  ... and {len(mismatches) - 5} more")

    return not mismatches


# --- 2. controller parity -------------------------------------------------

@dataclass
class _Sonar:
    state: str
    nearest_cm: Optional[int]
    distances: tuple = ()


def _random_obstacle(rng: random.Random) -> float:
    # The sonar guard's scale only multiplies the throttle; 0.0 (stopped) is
    # a deliberate divergence, checked in `run_obstacle_guard_check`.
    roll = rng.random()
    if roll < 0.6:
        return 1.0
    return rng.uniform(1e-3, 1.0)


def _feed(guard: ObstacleGuard, distances, dt: float = 0.1, state: str = "C", times: int = 1) -> float:
    for _ in range(times):
        guard.observe(_Sonar(state, None, tuple(distances)))
        guard.update(dt)
    return guard.scale


def run_obstacle_guard_check() -> bool:
    failures = []

    def expect(name: str, ok: bool) -> None:
        if not ok:
            failures.append(name)

    stop, slow = obstacle_module._STOP_CM, obstacle_module._SLOW_CM[1]
    release = obstacle_module._RELEASE_CM

    # Centre sensor: the old ramp, stop at the threshold.
    for d in (200, 150, 149, 120, 97, 80, 61):
        want = 1.0 if d >= slow else (d - stop) / (slow - stop)
        expect(f"ramp {d} cm", abs(_feed(ObstacleGuard(), (None, d, None), times=3) - want) < 1e-12)
    expect("stop at 60 cm", _feed(ObstacleGuard(), (None, 60, None), times=3) == 0.0)

    # Side sensors: ignored past their own slow distance (a person beside the
    # track), then the same ramp to the same stop, and they hold the stop.
    for index in (0, 2):
        side_slow = obstacle_module._SLOW_CM[index]
        for d in (side_slow, side_slow + 20, 149):
            distances = [None, None, None]
            distances[index] = d
            expect(f"side {index} at {d} cm does not slow", _feed(ObstacleGuard(), distances, times=3) == 1.0)
        for d in (side_slow - 10, (side_slow + stop) / 2, stop + 1):
            distances = [None, None, None]
            distances[index] = d
            want = (d - stop) / (side_slow - stop)
            expect(f"side {index} ramp {d} cm", abs(_feed(ObstacleGuard(), distances, times=3) - want) < 1e-12)
        for d in (30, 55, 60):
            distances = [None, None, None]
            distances[index] = d
            expect(f"side {index} at {d} cm stops", _feed(ObstacleGuard(), distances, times=3) == 0.0)
        guard = ObstacleGuard()
        distances = [None, None, None]
        distances[index] = 55
        _feed(guard, distances, times=3)
        distances[index] = 70
        expect(f"side {index} holds the stop", _feed(guard, distances, times=10) == 0.0)
    # The slowest-demanding sensor wins.
    expect("nearest ramp wins", abs(_feed(ObstacleGuard(), (80, 140, None), times=3) - (80 - stop) / (100 - stop)) < 1e-12)

    # Latch: a stationary obstacle wobbling around the threshold stays a stop.
    guard = ObstacleGuard()
    _feed(guard, (None, 58, None), times=3)
    wobble = [58, 64, 70, 62, 73, 59, 66, 74, 61] * 5
    expect("wobble stays stopped", all(_feed(guard, (None, d, None)) == 0.0 for d in wobble))
    # Release only after `_RELEASE_S` clear past `_RELEASE_CM`.
    dt = 0.1
    frames_needed = math.ceil(obstacle_module._RELEASE_S / dt)
    scales = [_feed(guard, (None, release + 5, None), dt=dt) for _ in range(frames_needed + 3)]
    released_at = next((i for i, v in enumerate(scales) if v > 0.0), None)
    # Two readings for the median to cross over, then the release time.
    expect("release waits", released_at is not None and released_at >= frames_needed)
    expect("release happens", released_at is not None and released_at <= frames_needed + 2)

    # One dropped echo while stopped, one ghost while cruising: no change.
    guard = ObstacleGuard()
    _feed(guard, (None, 50, None), times=3)
    expect("dropout ignored", _feed(guard, (None, None, None)) == 0.0 and _feed(guard, (None, 50, None)) == 0.0)
    guard = ObstacleGuard()
    _feed(guard, (None, 140, None), times=3)
    expect("ghost ignored", _feed(guard, (None, 30, None)) > 0.0)

    # An obstacle at the edge of a side beam echoes on some pings only: the
    # measured distance is held through the gaps, then forgotten.
    guard = ObstacleGuard()
    _feed(guard, (80, None, None), times=3)
    slowed = guard.scale
    expect("gap in echoes keeps the slow-down",
           slowed < 1.0 and all(_feed(guard, (None, None, None), dt=0.1) == slowed for _ in range(4)))
    expect("long gap forgets it", _feed(guard, (None, None, None), dt=0.1, times=3) == 1.0)

    # Stale data stops, and the release still waits.
    guard = ObstacleGuard()
    _feed(guard, (None, None, None), times=3)
    expect("stale stops", _feed(guard, (None, None, None), state="X") == 0.0)
    expect("stale release waits", _feed(guard, (None, None, None)) == 0.0)
    # No master at all (a replay): nothing to guard with.
    guard = ObstacleGuard()
    expect("no data is not gated", guard.update(0.1) == 1.0)

    # The controller holds the wheels while stopped, then resumes.
    auto = Autonomous()
    params = Config()
    held, _ = auto.step(120, 0.1, params, 0.05)
    during = [auto.step(offset, 0.1, params, 0.05, 0.0) for offset in (-300, 250, -80, 400)]
    expect("stop holds steering", all(s == held and t == 0.0 for s, t in during))
    expect("stop reason", auto.stop_reason == "obstacle")
    resumed, throttle = auto.step(120, 0.1, params, 0.05, 1.0)
    expect("resume steers again", throttle > 0.0 and auto.stop_reason is None)
    # Slowing never asks for less than the crawl the car moves at, so it
    # reaches the stop distance (and the hazards) instead of stalling short.
    crawl = params.min_speed * autonomous_module._SHARP_CURVE_FLOOR
    _s, throttle = Autonomous().step(0, 0.0, params, 0.05, 0.01)
    expect("slow zone keeps crawling", throttle == crawl)

    for name in failures:
        print(f"  obstacle guard: {name} failed")
    print(f"obstacle guard: {'all scenes pass' if not failures else f'{len(failures)} failures'}")
    return not failures


def run_controller_parity(baseline_module, steps: int = 6000) -> bool:
    rng = random.Random(_SEED)
    baseline = baseline_module.Autonomous()
    current = Autonomous()
    params = Config()
    mismatches = 0
    none_left = 0
    for step in range(steps):
        if step % 30 == 0:
            params = replace(Config(), kp=rng.uniform(0.0, 0.1), ki=rng.uniform(0.0, 0.05), kd=rng.uniform(0.0, 0.1),
                             base_speed=rng.uniform(0.0, 1.0), min_speed=rng.uniform(0.0, 1.0), scan_lines=rng.randint(1, 15))
        if none_left == 0 and rng.random() < 0.02:
            none_left = rng.randint(0, 300)
        if none_left > 0:
            none_left -= 1
            offset, curvature = None, None
        else:
            offset = rng.randint(-600, 600)
            curvature = None if rng.random() < 0.2 else rng.uniform(-3.0, 3.0)
        dt = rng.uniform(0.02, 0.08)
        obstacle = _random_obstacle(rng)
        if rng.random() < 0.005:
            baseline.reset()
            current.reset()
        steering, throttle = baseline.step(offset, curvature, params, dt, None)
        crawl = params.min_speed * autonomous_module._SHARP_CURVE_FLOOR
        want = (steering, max(throttle * obstacle, min(throttle, crawl)))
        # Alternate the bare call and the explicit defaults: both are Normal.
        if step % 2:
            got = current.step(offset, curvature, params, dt, obstacle)
        else:
            got = current.step(offset, curvature, params, dt, obstacle, profile=ControlProfile.NORMAL, provisional=rng.random() < 0.5)
        if want != got:
            mismatches += 1
            if mismatches <= 5:
                print(f"  step {step}: baseline {want} != current {got}")
    print(f"controller parity (NORMAL): {steps - mismatches}/{steps} steps identical")
    return mismatches == 0


# --- 3. calibration -------------------------------------------------------

def run_calibration_check(clip: Clip, limit) -> bool:
    tracker = LineTracker()
    params = Config()
    ready_at: Optional[int] = None
    budget = max(_CALIBRATION_BUDGET_FRAMES, int(2.0 * clip.fps()))
    for index, frame in frames(clip, min(limit or 10 ** 9, budget * 4)):
        tracker.process(frame, params)
        if tracker.profile_ready:
            ready_at = index
            break
    profile = tracker._width_profile
    verdict = "ok" if ready_at is not None and ready_at < budget else "FAIL"
    where = f"frame {ready_at}" if ready_at is not None else f"not within {budget * 4} frames"
    shape = f"width = {profile[0]:.3f}*y + {profile[1]:.1f}" if profile else "no profile"
    print(f"[{clip.name}] calibration at Config(): ready at {where} of a {budget}-frame budget ({shape}) -> {verdict}")
    return verdict == "ok"


# --- 4. learning guard ----------------------------------------------------

def run_learning_guard(clip: Clip, limit) -> bool:
    """The fork must not teach.

    A Normal pass at `Config()`. The working profile's slope is read when the
    fork opens (the first run of `_FORK_GAP_FRAMES` frames without a
    both-lines ROI row and fewer than two band rows after calibration) and
    must stay within 5 % of that value until the lane is measured again (a
    both-lines ROI row, or two band rows on the lane the car is now in).
    Before the fork the profile may legitimately move: on the hand-held clips
    the camera pitch drifts and the tracker follows it.
    """
    tracker = LineTracker()
    params = Config()
    fps = clip.fps()
    gap = 0
    opened_at: Optional[int] = None
    slope_at_open: Optional[float] = None
    worst = 0.0
    closed_at: Optional[int] = None
    for index, frame in frames(clip, limit):
        perc = tracker.process(frame, params)
        if not tracker.profile_ready:
            continue
        if perc.both_rows == 0 and perc.band_rows < 2:
            gap += 1
            if opened_at is None and gap >= _FORK_GAP_FRAMES:
                opened_at = index - gap + 1
                slope_at_open = tracker._width_profile[0]
            if opened_at is not None:
                worst = max(worst, abs(tracker._width_profile[0] / slope_at_open - 1.0))
        else:
            if opened_at is not None:
                closed_at = index
                break
            gap = 0
    if opened_at is None:
        print(f"[{clip.name}] learning guard: no fork gap found (never {_FORK_GAP_FRAMES} frames without the lane measured) -> not judged")
        return True
    verdict = "ok" if worst <= _LEARNING_GUARD_TOL else "FAIL"
    until = f"{closed_at / fps:.1f}s" if closed_at is not None else "end"
    print(f"[{clip.name}] learning guard: gap {opened_at / fps:.1f}s-{until}, slope {slope_at_open:.4f} at open, max drift {worst:.1%} -> {verdict}")
    return verdict == "ok"


# --- 5. operator replay ---------------------------------------------------

@dataclass(frozen=True)
class Script:
    name: str
    detour_at: Optional[float]      # seconds; None = never
    home_at: Optional[float]        # seconds; None = never ("forgot L1")

    def path_at(self, t: float) -> Path:
        path = Path.NORMAL
        if self.detour_at is not None and t >= self.detour_at:
            path = Path.DETOUR
        if self.home_at is not None and t >= self.home_at and (self.detour_at is None or self.home_at >= self.detour_at):
            path = HOME_PATH
        return path


# Where the operator would press R1 / L1 on the two hand-driven clips (times
# at the container fps, measured with tools/clip_census.py): the entry clip
# starts before the sign and is blind 6.9-8.3 s; the exit clip starts inside
# the detour, is blind 8.4-12.6 s through the merge and shows the main road
# again after the U-turn, so L1 comes late. "early" is inside the blind merge.
_SCRIPTS = {
    "detour_entry.mov": [
        Script("entry: R1 1 s, L1 16 s", 1.0, 16.0),
        Script("entry: forgot L1", 1.0, None),
        Script("entry: early L1 (7.2 s)", 1.0, 7.2),
    ],
    "detour_exit.mov": [
        Script("exit: R1 0 s, L1 21 s", 0.0, 21.0),
        Script("exit: forgot L1", 0.0, None),
        Script("exit: early L1 (9.5 s)", 0.0, 9.5),
    ],
}
_DEFAULT_SCRIPTS = [Script("R1 at 1/3, L1 at 2/3", None, None)]


def _scripts_for(clip: Clip, duration: float):
    scripts = _SCRIPTS.get(FilePath(clip.path).name)
    if scripts:
        return scripts
    return [Script("R1 at 1/3, L1 at 2/3", duration / 3, 2 * duration / 3), Script("forgot L1", duration / 3, None)]


def run_operator_replay(clip: Clip, limit) -> bool:
    fps = clip.fps()
    dt = 1.0 / fps
    cap = cv.VideoCapture(clip.path)
    count = int(cap.get(cv.CAP_PROP_FRAME_COUNT))
    cap.release()
    # Only the hand-driven clips have a script whose R1/L1 times match what
    # is on screen. On any other recording (a plain lap) Detour is armed at an
    # arbitrary time, so the budgets are reported but not judged.
    judged = FilePath(clip.path).name in _SCRIPTS
    ok = True
    for script in _scripts_for(clip, count / fps):
        passed = _replay_script(clip, script, dt, limit, judged)
        ok = (passed or not judged) and ok
    return ok


def _replay_script(clip: Clip, script: Script, dt: float, limit, judged: bool = True) -> bool:
    tracker = LineTracker()
    auto = Autonomous()
    fresh: Optional[LineTracker] = None
    params = Config()
    failures = []
    states = {}
    lost_run = 0
    longest_lost = {}
    hold_at_change = None
    max_release_delta = 0.0
    prev_steering = 0.0
    prev_path = Path.NORMAL
    prev_holding = 0.0
    release_trace = []
    reconverge = []
    home_frames = 0
    phase = "normal"
    seed_pending = False

    for index, frame in frames(clip, limit):
        t = index * dt
        path = script.path_at(t)
        if path is not prev_path:
            auto.seed_derivative()
            if prev_holding > 0:
                hold_at_change = (t, prev_holding, path)
            if path is Path.NORMAL and prev_path is Path.DETOUR:
                fresh = LineTracker()
                home_frames = 0
            prev_path = path
        phase = path.value
        perc = tracker.process(frame, params, anchor_for(path))
        was_holding = auto.holding > 0
        steering, throttle = auto.step(perc.lane_offset, perc.lane_curvature, params, dt,
                                       profile=profile_for(path), provisional=perc.provisional)
        holding_now = auto.holding > 0

        key = f"{phase}:{perc.lane_state.split()[0]}"
        states[key] = states.get(key, 0) + 1

        if perc.lane_offset is None:
            lost_run += 1
        else:
            if lost_run:
                longest_lost[phase] = max(longest_lost.get(phase, 0), lost_run)
            lost_run = 0

        if was_holding and not holding_now and auto.profile is ControlProfile.DETOUR:
            if perc.provisional:
                failures.append(f"t={t:.2f}s: a provisional frame ended a Detour hold")
            release_trace = [steering]
        elif release_trace and len(release_trace) < int(1.0 / dt):
            release_trace.append(steering)
            if len(release_trace) >= 2 and release_trace[0] * steering < 0:
                failures.append(f"t={t:.2f}s: steering reversed sign within 1 s of the hold release")
                release_trace = []

        if auto.profile is ControlProfile.DETOUR and perc.lane_offset is not None and not holding_now:
            delta = abs(steering - prev_steering)
            max_release_delta = max(max_release_delta, delta)
            if delta > autonomous_module._DETOUR_STEER_RATE * dt + 1e-9:
                failures.append(f"t={t:.2f}s: steering step {delta:.3f} exceeds the slew {autonomous_module._DETOUR_STEER_RATE * dt:.3f}")

        if hold_at_change is not None:
            t0, held, new_path = hold_at_change
            if auto.holding == 0 and perc.lane_offset is None and auto.stop_reason is None:
                pass
            if not holding_now and perc.lane_offset is None and auto.stop_reason == "lane lost" and t - t0 < autonomous_module._DETOUR_HOLD_S - held - 1e-6:
                failures.append(f"t={t:.2f}s: hold started before the path change to {new_path.value} at {t0:.2f}s was cut short")
                hold_at_change = None
            if perc.lane_offset is not None and not holding_now:
                hold_at_change = None

        if fresh is not None:
            fresh_perc = fresh.process(frame, params)
            home_frames += 1
            if home_frames == _RECONVERGE_FRAMES:
                a, b = pid_error(perc), pid_error(fresh_perc)
                if a is not None and b is not None:
                    reconverge.append(abs(a - b))
                    if abs(a - b) > _RECONVERGE_TOL_PX:
                        failures.append(f"t={t:.2f}s: Normal after Detour differs from a fresh Normal tracker by {abs(a - b):.1f} px after {_RECONVERGE_FRAMES} frames")
                fresh = None

        prev_steering = steering
        prev_holding = auto.holding

    if lost_run:
        longest_lost[phase] = max(longest_lost.get(phase, 0), lost_run)
    for run_phase, run in longest_lost.items():
        if run * dt > autonomous_module._DETOUR_HOLD_S and run_phase == Path.DETOUR.value:
            failures.append(f"LOST run of {run * dt:.1f}s in {run_phase} exceeds the {autonomous_module._DETOUR_HOLD_S:.0f}s hold")

    print(f"[{clip.name}] operator replay '{script.name}':")
    total = sum(states.values())
    for key, n in sorted(states.items(), key=lambda kv: (-kv[1], kv[0]))[:8]:
        print(f"    {key:32s} {n:5d} ({n / total:.0%})")
    print("    longest LOST run per phase: " + ", ".join(f"{p} {n * dt:.1f}s" for p, n in longest_lost.items()) if longest_lost else "    no LOST frames")
    print(f"    max steering step while reading in Detour: {max_release_delta:.3f} (budget {autonomous_module._DETOUR_STEER_RATE * dt:.3f})")
    if reconverge:
        print(f"    Normal-after-Detour vs fresh tracker after {_RECONVERGE_FRAMES} frames: {reconverge[0]:.1f} px")
    for failure in failures[:6]:
        print(f"    FAIL: {failure}")
    if len(failures) > 6:
        print(f"    ... and {len(failures) - 6} more")
    verdict = 'ok' if not failures else 'FAIL'
    print(f"    -> {verdict}" + ("" if judged else " (informational: no scripted R1/L1 times for this recording)"))
    return not failures


# --- 5b. automatic return -------------------------------------------------

# (arm time in the video's own clock, latest acceptable return, merge window)
# The day-1 recording is tagged at 30 fps while the loop ran at 24.6, so its
# live-CSV times are scaled by 0.822 here; the merge was blind 34.2-35.6 s.
_RETURN_SCRIPTS = {
    "day1_detour.mp4": (23.5, 34.0, (33.5, 36.5)),
    "detour_entry.mov": (1.0, 17.0, None),
}


def run_auto_return(clip: Clip, limit) -> bool:
    """Armed at the sign, the return must fire after the entrance gap and
    before the merge, and the carried hold must take the car through the
    merge without a stop."""
    script = _RETURN_SCRIPTS.get(FilePath(clip.path).name)
    if script is None:
        return True
    arm_at, latest, merge = script
    dt = 1.0 / clip.fps()
    tracker, auto, watch = LineTracker(), Autonomous(), DetourReturn()
    params = Config()
    path = HOME_PATH
    armed = returned_at = gap_at = None
    stops = []
    for index, frame in frames(clip, limit):
        t = index * dt
        if armed is None and t >= arm_at:
            armed, path = t, Path.DETOUR
        perc = tracker.process(frame, params, anchor_for(path))
        home = watch.update(path, perc, dt)
        if gap_at is None and watch.stage.startswith("inside"):
            gap_at = t
        if home is not None:
            returned_at, path = t, home
        auto.step(perc.lane_offset, perc.lane_curvature, params, dt,
                  profile=watch.profile_for(path), provisional=perc.provisional)
        watch.observe_hold(auto.holding, dt)
        if auto.stop_reason and (not stops or t - stops[-1] > 1.0):
            stops.append(t)
    failures = []
    if returned_at is None:
        failures.append("never returned home")
    else:
        if gap_at is None or returned_at <= gap_at:
            failures.append("returned before the entrance gap")
        if returned_at > latest:
            failures.append(f"returned at {returned_at:.1f}s, after the {latest:.1f}s limit")
    if merge is not None:
        in_merge = [t for t in stops if merge[0] <= t <= merge[1]]
        if in_merge:
            failures.append(f"stopped in the merge at {in_merge[0]:.1f}s")
    print(f"[{clip.name}] auto return: armed {arm_at:.1f}s, gap {gap_at if gap_at is None else round(gap_at, 1)}s,"
          f" home at {returned_at if returned_at is None else round(returned_at, 1)}s, stops at {[round(t, 1) for t in stops]}"
          f" -> {'ok' if not failures else 'FAIL: ' + '; '.join(failures)}")
    return not failures


# --- 6. bias --------------------------------------------------------------

# The bug this guards against was a 0.6 curvature tilt (42 px). On clean
# frames the row profile should leave a few px and well under 0.1 curvature.
_BIAS_CURVATURE_TOL = 0.1
_BIAS_ERROR_TOL_PX = 10.0
# A "whole lane" row: the anchored scan measured the other edge within this
# ratio of the width it expects. The far row regularly reaches across to a
# line beyond the lane (1.5x on road.mov); Normal's midpoint is wrong there
# and cannot be a reference.
_CLEAN_WIDTH_RATIO = 1.15


def run_bias_check(clip: Clip, limit, judge: bool) -> bool:
    """Straight (LEFT) must see the same road Normal sees, where Normal sees it.

    "Clean" frames are those where every row Normal resolved saw both lines
    (two or more of them; a single-edge row uses the scalar width and tilts
    Normal's own curvature) and Straight measured a whole lane on every row.
    On them the two-edge midpoint is a reference. Row-wise, each both-lines
    row's centre is compared too, over all frames. Reported for every clip; pass/fail only on
    recordings from the car's own pipeline (`judge`), since the clips are
    hand-driven screen recordings with a moving camera.
    """
    params = Config()
    normal = LineTracker()
    straight = LineTracker()
    diffs, clean, row_diffs = [], [], []
    curv, curv_ref = [], []
    throttles = {"normal": [], "straight": []}
    controller = {"normal": Autonomous(), "straight": Autonomous()}
    dt = 1.0 / clip.fps()

    for index, frame in frames(clip, limit):
        ref = normal.process(frame, params)
        got = straight.process(frame, params, LaneAnchor.LEFT)
        for label, perc in (("normal", ref), ("straight", got)):
            _s, thr = controller[label].step(perc.lane_offset, perc.lane_curvature, params, dt)
            throttles[label].append(thr)
        a, b = pid_error(ref), pid_error(got)
        if a is None or b is None or not got.profile_ready or got.lane_state.startswith("calibrating"):
            continue
        diffs.append(b - a)
        both = {y: c for y, c, is_both in ref.row_centers if is_both}
        for y, c, _ in got.row_centers:
            if y in both:
                row_diffs.append(c - both[y])
        ratios = got.width_ratios
        whole_lane = len(ratios) == len(got.row_centers) and all(1.0 / _CLEAN_WIDTH_RATIO <= r <= _CLEAN_WIDTH_RATIO for r in ratios)
        if ref.both_rows >= 2 and ref.both_rows == ref.rows_found and whole_lane:
            clean.append(b - a)
            curv.append(got.lane_curvature or 0.0)
            curv_ref.append(ref.lane_curvature or 0.0)

    print(f"[{clip.name}] bias, Straight (LEFT) against Normal at Config():")
    if not clean:
        print("    no clean frames (Normal never resolved every row from both lines while Straight measured a whole lane) -> " + ("FAIL" if judge else "not judged"))
        return not judge
    mean_diff = mean(clean)
    p95 = sorted(abs(d) for d in clean)[int(0.95 * (len(clean) - 1))]
    dcurv = abs(mean(curv) - mean(curv_ref))
    print(f"    all frames   n={len(diffs):4d}: mean {mean(diffs):+6.1f} px")
    print(f"    clean frames n={len(clean):4d}: mean {mean_diff:+6.1f} px, p95 {p95:5.1f} px, curvature {mean(curv):+.3f} vs Normal {mean(curv_ref):+.3f}")
    if row_diffs:
        print(f"    row-wise centre difference on both-lines rows: mean {mean(row_diffs):+.1f} px, n={len(row_diffs)}")
    print(f"    implied throttle: Normal {mean(throttles['normal']):.3f}, Straight {mean(throttles['straight']):.3f} (base {params.base_speed})")
    passed = abs(mean_diff) <= _BIAS_ERROR_TOL_PX and dcurv <= _BIAS_CURVATURE_TOL
    print(f"    -> {'ok' if passed else 'BIASED'}" + ("" if judge else " (informational on a hand-driven clip)"))
    return passed or not judge


# --- 7. smoke -------------------------------------------------------------

def run_anchor_smoke(clip: Clip, side: LaneAnchor, limit, maker=random_params) -> bool:
    tracker = LineTracker()
    states = Counter()
    fallback_failures = 0
    provisional = 0

    for index, frame, params in replay(clip, limit, maker):
        try:
            result = tracker.process(frame, params, side)
        except Exception as error:
            if raised_in(error, "_scan_anchored") or raised_in(error, "_scan_band"):
                print(f"[{clip.name}] {side.value}: frame {index} raised in the anchored/band scan: {error!r}")
                return False
            # The AUTO scan, which runs until the width is calibrated and as
            # the fail-safe, carries its pre-existing out-of-frame scan with it.
            fallback_failures += 1
            tracker.reset()
            continue
        states[result.lane_state.split()[0]] += 1
        provisional += result.provisional
        profile = tracker._width_profile
        if profile is not None and not all(math.isfinite(v) for v in profile):
            print(f"[{clip.name}] {side.value}: frame {index} left a non-finite width profile {profile}")
            return False

    total = sum(states.values())
    histogram = ", ".join(f"{state} {count / total:.0%}" for state, count in states.most_common())
    print(f"[{clip.name}] {side.value} smoke (wide sliders): {total} frames, no exception from the anchored scan"
          f" ({fallback_failures} pre-existing AUTO out-of-frame failures), {provisional} provisional, {tracker.clamped_frames} clamped")
    print(f"    {histogram}")
    return True


# --- main -----------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("videos", nargs="+", help="clips and recordings to replay")
    parser.add_argument("--flip", action="append", default=[], metavar="VIDEO", help="mirror this clip left-right (repeatable)")
    parser.add_argument("--ref", default=_DEFAULT_REF, help="git ref holding the pre-change tracker and controller")
    parser.add_argument("--limit", type=int, help="stop after this many frames per clip")
    parser.add_argument("--judge-bias", action="append", default=[], metavar="VIDEO",
                        help="make the bias check pass/fail on this recording (the day-0 Normal lap)")
    parser.add_argument("--skip", default="", help="comma-separated checks to skip: auto,controller,calibration,learning,replay,bias,smoke")
    args = parser.parse_args()

    skip = set(filter(None, args.skip.split(",")))
    clips = [Clip(video, video in args.flip) for video in args.videos]
    baseline_tracker = load_baseline(args.ref, "src/perception/line_tracking.py", "baseline_line_tracking")
    baseline_controller = load_baseline(args.ref, "src/input/autonomous.py", "baseline_autonomous")

    ok = True
    if "auto" not in skip:
        for clip in clips:
            ok = run_auto_parity(clip, baseline_tracker, args.limit) and ok
            ok = run_auto_parity(clip, baseline_tracker, args.limit, tuned_params) and ok
    if "controller" not in skip:
        ok = run_controller_parity(baseline_controller) and ok
        ok = run_obstacle_guard_check() and ok
    if "calibration" not in skip:
        for clip in clips:
            ok = run_calibration_check(clip, args.limit) and ok
    if "learning" not in skip:
        for clip in clips:
            ok = run_learning_guard(clip, args.limit) and ok
    if "replay" not in skip:
        for clip in clips:
            ok = run_operator_replay(clip, args.limit) and ok
            ok = run_auto_return(clip, args.limit) and ok
    if "bias" not in skip:
        for clip in clips:
            ok = run_bias_check(clip, args.limit, judge=clip.path in args.judge_bias) and ok
    if "smoke" not in skip:
        for clip in clips:
            for side in (LaneAnchor.LEFT, LaneAnchor.RIGHT):
                ok = run_anchor_smoke(clip, side, args.limit) and ok

    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
