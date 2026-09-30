#!/usr/bin/env python3
"""Closed-loop synthetic-track harness for the lane tracker and the controller.

The 3d-simulator's SVG track (a stadium loop with two right-hand detours that
cut across its interior) is rendered from a pin-hole camera riding a kinematic
bicycle; every frame goes through `LineTracker.process` and `Autonomous.step`
and the resulting command, after a frame delay and first-order lags, moves
the bicycle. A scripted operator presses R1 (Detour) at the sign before the
fork and L1 (home) once the tracker reads a whole lane again after the merge.

This is a go/no-go ranking harness, not a predictor: it says whether a change
to the tracker or the controller makes the car cross a line, run out its hold
or come out of a blind stretch pointing the wrong way, relative to the same
run before the change. Nothing in it is calibrated yet: every physical number
is a PLACEHOLDER (see `--print-params`) until measured on the car:

    wheelbase, full-lock turning radius, ground speed at throttle 0.375/0.50/0.63,
    steering and motor time constants, command delay in frames,
    camera height, pitch, focal length (FOV) and barrel k1,
    the taped lane width (line centre to centre) on the main road and detours,
    the fork fillet radius and merge angle as taped.

Usage:

    python tools/synth_track.py                                  # all scenarios, manila camera
    python tools/synth_track.py --scenario loop-normal --check   # exit 1 on a regression
    python tools/synth_track.py --scenario detour-easy --camera car --out /tmp/d.mp4
    python tools/synth_track.py --sweep --camera car             # robustness ranking table
    python tools/synth_track.py --replay-census assets/road.mov  # sim vs recording census
    python tools/synth_track.py --fit-widths 0.62,-71.0 --camera car
    python tools/synth_track.py --print-params

Scenarios: loop-normal, loop-straight, straight-both-forks, detour-easy,
detour-hard, forgot-L1, early-L1. `--check` fails on any line crossing, a
LOST run at or over the controller's hold, a heading error at re-acquisition
over 45 deg, a run that stopped or left the road, and on loop-straight
drifting from loop-normal by more than 10 px of lateral error or 10 % of
steering sign-change rate.

`--replay-census` is the gate before any constant is read off this sim: the
sim's loop-normal must produce the same rows_found / both_rows histograms
and offset / curvature quantiles as a real recording, within the budgets at
the top of this file.
"""

import argparse
import math
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field, replace
from pathlib import Path as FsPath
from typing import Optional

import cv2 as cv
import numpy as np

REPO_ROOT = FsPath(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from config import Config  # noqa: E402
from perception.line_tracking import LaneAnchor, LineTracker  # noqa: E402
import input.autonomous as autonomous_mod  # noqa: E402
from input.autonomous import Autonomous, _PIDController  # noqa: E402
from path import HOME_PATH, Path, anchor_for, profile_for  # noqa: E402

# --- budgets (go/no-go) -----------------------------------------------------
_HEADING_REACQ_BUDGET_DEG = 45.0   # heading error when a hold ends
_STRAIGHT_VS_NORMAL_PX = 10.0      # loop-straight lateral error vs loop-normal
_STRAIGHT_VS_NORMAL_RATE = 0.10    # steering sign-change rate, relative
_STRAIGHT_VS_NORMAL_RATE_FLOOR = 0.05  # absolute floor for the rate comparison (1/s)
_FIT_WIDTH_BUDGET = 0.05           # --fit-widths max relative error
_CENSUS_ROWS_BUDGET = 0.15         # rows histograms, per bin, fraction
_CENSUS_OFFSET_BUDGET_PX = 15.0    # offset p50
_CENSUS_CURVATURE_BUDGET = 0.10    # curvature p50
_STOP_BUDGET_S = 1.0               # controller stopped this long -> run over
_OFF_ROAD_M = 1.0                  # lateral error this large -> run over
_STEER_DEADBAND = 0.05             # |steering| below this has no sign
_SIGN_AHEAD_M = 1.0                # the sign stands this far before the gap opens
_TAIL_AFTER_UTURN_M = 0.3          # detour scenarios end this far after the next U-turn
_STEP_M = 0.02                     # polyline spacing
_GROUND_GREY = 40
_TAPE_GREY = 235

SCENARIOS = ["loop-normal", "loop-straight", "straight-both-forks", "detour-easy", "detour-hard", "forgot-L1", "early-L1"]


# --- camera -------------------------------------------------------------------

@dataclass
class CameraModel:
    name: str
    width: int
    height: int
    focal: float        # px, full-resolution frame
    horizon_row: float  # principal point row: the horizon when pitch is 0
    height_m: float     # camera above the ground
    pitch_deg: float    # looking down, positive
    k1: float           # radial distortion, negative = barrel
    placeholder: bool = False

    @property
    def pitch(self) -> float:
        return math.radians(self.pitch_deg)

    def project(self, pts, pose, tape_m):
        """World (x, y) points -> (u, v, line width px, valid). Pose is the
        camera's (x, y, heading); heading turns the camera right by +heading
        (y grows downward on the plan, as in the SVG)."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        x0, y0, heading = pose
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        dx = pts[:, 0] - x0
        dy = pts[:, 1] - y0
        fwd = dx * cos_h + dy * sin_h
        right = -dx * sin_h + dy * cos_h
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        yp = self.height_m * cp - fwd * sp
        zp = self.height_m * sp + fwd * cp
        valid = zp > 0.05
        zs = np.where(valid, zp, 1.0)
        xn = right / zs
        yn = yp / zs
        r2 = xn * xn + yn * yn
        if self.k1 < 0:
            valid &= r2 < 0.9 * (-1.0 / (3.0 * self.k1))
        d = 1.0 + self.k1 * r2
        u = self.width / 2 + self.focal * xn * d
        v = self.horizon_row + self.focal * yn * d
        lim = 4.0 * max(self.width, self.height)
        u = np.clip(u, -lim, lim)
        v = np.clip(v, -lim, lim)
        w = np.clip(self.focal * tape_m / zs, 1, self.width).astype(int)
        return u, v, w, valid

    def render(self, lines, pose, tape_m):
        frame = np.full((self.height, self.width, 3), _GROUND_GREY, dtype=np.uint8)
        for line in lines:
            u, v, w, valid = self.project(line, pose, tape_m)
            pts = []
            for ok, uu, vv, ww in zip(valid, u, v, w):
                if not ok:
                    if pts:
                        _draw(frame, pts)
                        pts = []
                    continue
                pts.append((int(round(uu)), int(round(vv)), int(ww)))
            if pts:
                _draw(frame, pts)
        return frame


def _draw(frame, pts):
    for (u0, v0, w0), (u1, v1, _w1) in zip(pts, pts[1:]):
        cv.line(frame, (u0, v0), (u1, v1), (_TAPE_GREY, _TAPE_GREY, _TAPE_GREY), w0, cv.LINE_AA)


CAMERAS = {
    # The manila open-loop tool's camera, verbatim.
    "manila": CameraModel("manila", 1274, 746, 800.0, 250.0, 0.5, 0.0, 0.0),
    # PLACEHOLDER: the car's camera. Frame size is real (2700x1536, the
    # tracker downscales by 2). Focal, pitch, height and k1 are chosen so that
    # the lane spans ~90 % of the frame at row 0.67*h and the two nearest ROI
    # rows (>= 0.87*h) see no line, which is what the recordings show. Replace
    # with measured values (height, pitch, horizontal FOV, k1).
    "car": CameraModel("car", 2700, 1536, 900.0, 768.0, 0.18, 52.0, -0.08, placeholder=True),
}


# --- car ----------------------------------------------------------------------

@dataclass
class CarModel:
    # PLACEHOLDER values throughout; see `--print-params`.
    wheelbase_m: float = 0.25
    r_lock_m: float = 0.60                         # turning radius at full lock
    speed_table: tuple = ((0.375, 0.45), (0.50, 0.60), (0.63, 0.80))  # (throttle, m/s)
    steer_lag_s: float = 0.15
    motor_lag_s: float = 0.30
    delay_frames: int = 2
    camera_ahead_m: float = 0.15                   # camera in front of the rear axle

    @property
    def max_steer(self) -> float:
        return math.atan(self.wheelbase_m / self.r_lock_m)

    def speed_for(self, throttle: float) -> float:
        t = [0.0] + [p[0] for p in self.speed_table]
        v = [0.0] + [p[1] for p in self.speed_table]
        if throttle <= t[-1]:
            return float(np.interp(throttle, t, v))
        slope = (v[-1] - v[-2]) / (t[-1] - t[-2])
        return max(0.0, v[-1] + slope * (throttle - t[-1]))


class Bicycle:
    """Kinematic bicycle (3d-simulator's tools/physics.py without its friction
    hack): heading += v / R * dt, R = wheelbase / tan(delta)."""

    def __init__(self, car: CarModel, x: float, y: float, heading: float):
        self.car = car
        self.x, self.y, self.heading = x, y, heading
        self.speed = 0.0
        self.delta = 0.0

    @property
    def pose(self):
        return (self.x, self.y, self.heading)

    def camera_pose(self):
        a = self.car.camera_ahead_m
        return (self.x + a * math.cos(self.heading), self.y + a * math.sin(self.heading), self.heading)

    def step(self, steering: float, throttle: float, dt: float) -> None:
        if dt <= 0:
            return
        target_speed = self.car.speed_for(max(0.0, throttle))
        self.speed += (target_speed - self.speed) * min(1.0, dt / max(1e-6, self.car.motor_lag_s))
        target_delta = max(-1.0, min(1.0, steering)) * self.car.max_steer
        self.delta += (target_delta - self.delta) * min(1.0, dt / max(1e-6, self.car.steer_lag_s))
        omega = self.speed * math.tan(self.delta) / self.car.wheelbase_m
        self.heading += omega * dt
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt


# --- geometry helpers ------------------------------------------------------

def _ease(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def _offset_curve(centre, offset):
    """Points offset laterally (to the right for +offset) from a polyline.
    `offset` may be a number or a function of the point's z."""
    out = []
    for i, (x, z) in enumerate(centre):
        x0, z0 = centre[max(i - 1, 0)]
        x1, z1 = centre[min(i + 1, len(centre) - 1)]
        dx, dz = x1 - x0, z1 - z0
        n = math.hypot(dx, dz) or 1.0
        o = offset(z) if callable(offset) else offset
        # right-hand normal of the direction (dx, dz) is (dz, -dx)
        out.append((x + o * dz / n, z - o * dx / n))
    return out


def _bezier(p0, p1, p2, p3, n=200):
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    u = 1.0 - t
    P = np.array([p0, p1, p2, p3], float)
    return u ** 3 * P[0] + 3 * u ** 2 * t * P[1] + 3 * u * t ** 2 * P[2] + t ** 3 * P[3]


def _arc(cx, cy, r, a0, a1, n=64):
    a = np.linspace(a0, a1, n + 1)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], 1)


def _cumlen(poly):
    seg = np.hypot(*np.diff(poly, axis=0).T)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _resample(poly, step):
    poly = np.asarray(poly, float)
    s = _cumlen(poly)
    if s[-1] < step:
        return poly
    q = np.append(np.arange(0.0, s[-1], step), s[-1])
    return np.stack([np.interp(q, s, poly[:, 0]), np.interp(q, s, poly[:, 1])], 1)


def _point_at(poly, s_vals, s):
    return np.array([np.interp(s, s_vals, poly[:, 0]), np.interp(s, s_vals, poly[:, 1])])


def _fillet_join(a, b, radius):
    """Join polyline `a` (ending at the corner) to `b` (starting there) with a
    tangent-continuous round of the given radius (a quadratic bezier whose
    control point is the corner; exact circle is not needed for a tape line)."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    if radius <= 0:
        return np.vstack([a, b[1:]])
    corner = (a[-1] + b[0]) / 2
    da = a[-1] - a[-2]
    db = b[1] - b[0]
    da /= np.linalg.norm(da) or 1.0
    db /= np.linalg.norm(db) or 1.0
    theta = math.acos(max(-1.0, min(1.0, float(np.dot(da, db)))))
    if theta < 1e-3:
        return np.vstack([a, b[1:]])
    d = radius * math.tan(theta / 2)
    sa = _cumlen(a)
    ta = max(sa[-1] - d, 0.0)
    ia = int(np.searchsorted(sa, ta))
    pa = _point_at(a, sa, ta)
    sb = _cumlen(b)
    tb = min(d, sb[-1])
    ib = int(np.searchsorted(sb, tb, side="right"))
    pb = _point_at(b, sb, tb)
    n = max(6, int(2 * d / _STEP_M))
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    curve = (1 - t) ** 2 * pa + 2 * (1 - t) * t * corner + t ** 2 * pb
    return np.vstack([a[:ia], curve, b[ib:]])


def _hermite(p0, t0, p1, t1, n=40):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = np.linalg.norm(p1 - p0)
    t0 = np.asarray(t0, float) / (np.linalg.norm(t0) or 1.0) * L
    t1 = np.asarray(t1, float) / (np.linalg.norm(t1) or 1.0) * L
    s = np.linspace(0.0, 1.0, n + 1)[:, None]
    h00 = 2 * s ** 3 - 3 * s ** 2 + 1
    h10 = s ** 3 - 2 * s ** 2 + s
    h01 = -2 * s ** 3 + 3 * s ** 2
    h11 = s ** 3 - s ** 2
    return h00 * p0 + h10 * t0 + h01 * p1 + h11 * t1


def _nearest_index(poly, p):
    d = np.hypot(poly[:, 0] - p[0], poly[:, 1] - p[1])
    return int(np.argmin(d))


def _rotate_closed(poly, p):
    """Closed polyline (first == last) re-started at the point nearest `p`."""
    body = poly[:-1]
    i = _nearest_index(body, p)
    rolled = np.roll(body, -i, axis=0)
    return np.vstack([rolled, rolled[:1]])


def _signed_lateral(poly, p, lo=0, hi=None):
    """(signed distance, segment index): positive when `p` is right of the
    polyline's direction (y grows downward)."""
    a = poly[lo:hi][:-1]
    b = poly[lo:hi][1:]
    ab = b - a
    ap = np.asarray(p, float) - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    t = np.clip((ap * ab).sum(1) / L2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d = np.hypot(*(np.asarray(p, float) - proj).T)
    i = int(np.argmin(d))
    cross = ab[i, 0] * ap[i, 1] - ab[i, 1] * ap[i, 0]
    return float(d[i]) * (1.0 if cross >= 0 else -1.0), lo + i


class Route:
    """Open polyline with arclength; the loop routes end where they start."""

    def __init__(self, pts):
        self.pts = np.asarray(pts, float)
        self.s = _cumlen(self.pts)
        self.length = float(self.s[-1])

    def advance(self, prev_idx, p, back=30, ahead=120):
        lo = max(0, prev_idx - back)
        hi = min(len(self.pts), prev_idx + ahead)
        e, i = _signed_lateral(self.pts, p, lo, hi)
        d = self.pts[min(i + 1, len(self.pts) - 1)] - self.pts[i]
        return i, float(self.s[i]), e, math.atan2(d[1], d[0])

    def index_of_s(self, s):
        return int(np.searchsorted(self.s, s))

    def nearest_s(self, p):
        return float(self.s[_nearest_index(self.pts, p)])


# --- track ----------------------------------------------------------------------

_DP1 = (((2345, 3057.5), (2695.09, 2247.6), (2708.21, 1788.7), (2345, 960)),
        ((3056, 3057.5), (3406.09, 2247.6), (3419.21, 1788.7), (3056, 960)))
_DP2 = (((4873, 3050.5), (5223.09, 2240.6), (5236.21, 1781.7), (4873, 953)),
        ((5584, 3050.5), (5934.09, 2240.6), (5947.21, 1781.7), (5584, 953)))


@dataclass
class TrackModel:
    """The 3d-simulator SVG constants (Group 25.svg), in SVG units."""
    outer_radius: float = 1693.0
    inner_radius: float = 1045.25
    outer_left_cx: float = 1704.5
    outer_right_cx: float = 6491.5
    inner_left_cx: float = 1810.0
    inner_right_cx: float = 6378.5
    y_top_outer: float = 312.0
    y_bot_outer: float = 3698.0
    y_top_inner: float = 960.0
    y_bot_inner: float = 3050.5
    lane_svg: float = 648.0               # line centre to centre on the straights
    detours: tuple = (_DP1, _DP2)         # (car's right line, car's left line) bezier pairs, bottom->top
    taped_lane_m: float = 0.50            # PLACEHOLDER: measured taped lane width
    tape_m: float = 0.05                  # PLACEHOLDER: tape width
    fork_fillet_m: float = 0.5            # PLACEHOLDER: radius taped at the fork / merge corners
    merge_angle_deg: Optional[float] = None  # None = the bezier's own (~66 deg)
    width_factor: float = 1.0             # detour width relative to the SVG pair
    start_x_svg: float = 6000.0           # start on the bottom straight, heading west

    @property
    def scale(self) -> float:
        return self.taped_lane_m / self.lane_svg


@dataclass
class Detour:
    name: str
    right_line: np.ndarray     # car's right line, fork -> merge (filleted), metres
    island_line: np.ndarray    # car's left line, wedge top -> wedge bottom
    centre: np.ndarray         # pointwise mid of the raw pair, top -> bottom
    x_fork: float
    x_wedge: float


class Track:
    def __init__(self, model: TrackModel):
        self.model = model
        m = model
        S = m.scale
        self.S = S
        yt_c = (m.y_top_outer + m.y_top_inner) / 2
        yb_c = (m.y_bot_outer + m.y_bot_inner) / 2
        self.cy = (yt_c + yb_c) / 2 * S
        self.cx_l = (m.outer_left_cx + m.inner_left_cx) / 2 * S
        self.cx_r = (m.outer_right_cx + m.inner_right_cx) / 2 * S
        self.y_top_inner = m.y_top_inner * S
        self.y_bot_inner = m.y_bot_inner * S
        self.y_top_outer = m.y_top_outer * S
        self.y_bot_outer = m.y_bot_outer * S
        self.lane_m = m.taped_lane_m
        self.start = (m.start_x_svg * S, yb_c * S, math.pi)

        def capsule(lcx, rcx, yt, yb, r):
            cy = (yt + yb) / 2
            top = np.array([[lcx, yt], [rcx, yt]])
            right = _arc(rcx, cy, r, -math.pi / 2, math.pi / 2)
            bot = np.array([[rcx, yb], [lcx, yb]])
            left = _arc(lcx, cy, r, math.pi / 2, 3 * math.pi / 2)
            return _resample(np.vstack([top, right[1:], bot, left[1:]]), _STEP_M / S)

        outer_svg = capsule(m.outer_left_cx, m.outer_right_cx, m.y_top_outer, m.y_bot_outer, m.outer_radius)
        inner_svg = capsule(m.inner_left_cx, m.inner_right_cx, m.y_top_inner, m.y_bot_inner, m.inner_radius)
        centre_svg = capsule((m.outer_left_cx + m.inner_left_cx) / 2, (m.outer_right_cx + m.inner_right_cx) / 2,
                             yt_c, yb_c, (m.outer_radius + m.inner_radius) / 2)

        # Detours: pair sorted by x, snapped onto the inner line, width-scaled,
        # merge angle applied, travelled top -> bottom.
        dets = []
        for pair in m.detours:
            a, b = sorted(pair, key=lambda ctrl: ctrl[0][0])
            right = [list(p) for p in reversed(a)]
            island = [list(p) for p in reversed(b)]
            for ctrl in (right, island):
                ctrl[0][1] = m.y_top_inner
                ctrl[3][1] = m.y_bot_inner
                if m.merge_angle_deg is not None:
                    L = math.hypot(ctrl[2][0] - ctrl[3][0], ctrl[2][1] - ctrl[3][1])
                    ang = math.radians(m.merge_angle_deg)
                    ctrl[2] = [ctrl[3][0] + L * math.cos(ang), ctrl[3][1] - L * math.sin(ang)]
            r_pts = _bezier(*right)
            i_pts = _bezier(*island)
            mid = (r_pts + i_pts) / 2
            half = (i_pts - r_pts) / 2
            r_pts = mid - m.width_factor * half
            i_pts = mid + m.width_factor * half
            dets.append((r_pts, i_pts, mid))
        dets.sort(key=lambda d: d[0][0, 0])

        # Inner line pieces: between consecutive detours the inner line, the
        # next detour's right line and the previous detour's island form a loop.
        n = len(dets)
        loops = []
        fillet = m.fork_fillet_m / S
        step = _STEP_M / S
        cy_i = (m.y_top_inner + m.y_bot_inner) / 2
        for k in range(n + 1):
            xa = dets[k - 1][1][0, 0] if k > 0 else m.inner_left_cx
            xb = dets[k][0][0, 0] if k < n else m.inner_right_cx
            top = _resample(np.array([[xa, m.y_top_inner], [xb, m.y_top_inner]]), step)
            if k < n:
                poly = _fillet_join(top, _resample(dets[k][0], step), fillet)
                bot = _resample(np.array([[xb, m.y_bot_inner], [xa, m.y_bot_inner]]), step)
                poly = _fillet_join(poly, bot, fillet)
            else:
                right = _arc(m.inner_right_cx, cy_i, m.inner_radius, -math.pi / 2, math.pi / 2)
                bot = np.array([[m.inner_right_cx, m.y_bot_inner], [xa, m.y_bot_inner]])
                poly = np.vstack([top, right[1:], bot])
            if k > 0:
                poly = np.vstack([poly, dets[k - 1][1][::-1][1:]])
            else:
                left = _arc(m.inner_left_cx, cy_i, m.inner_radius, math.pi / 2, 3 * math.pi / 2)
                poly = np.vstack([poly, left[1:]])
            loops.append(_resample(poly, step))

        self.lines = [outer_svg * S] + [lp * S for lp in loops]
        self.outer = _rotate_closed(outer_svg * S, self.start[:2])
        self.inner = _rotate_closed(inner_svg * S, self.start[:2])
        self.centre = _rotate_closed(centre_svg * S, self.start[:2])

        # Name detours by how far their merge is from the next U-turn.
        infos = []
        for r_pts, i_pts, mid in dets:
            x_fork = r_pts[0, 0]
            right_line = None
            for lp in loops:
                pass
            infos.append((x_fork - m.inner_left_cx, r_pts, i_pts, mid))
        infos.sort(key=lambda t: t[0])
        names = ["hard", "easy"] if len(infos) == 2 else [f"d{i}" for i in range(len(infos))]
        self.detours = []
        for name, (_dist, r_pts, i_pts, mid) in zip(names, infos):
            top = _resample(np.array([[r_pts[0, 0] - 2 * fillet, m.y_top_inner], [r_pts[0, 0], m.y_top_inner]]), step)
            bot = _resample(np.array([[r_pts[0, 0], m.y_bot_inner], [r_pts[0, 0] - 2 * fillet, m.y_bot_inner]]), step)
            rl = _fillet_join(_fillet_join(top, _resample(r_pts, step), fillet), bot, fillet)
            self.detours.append(Detour(name, _resample(rl, step) * S, _resample(i_pts, step) * S, _resample(mid, step) * S,
                                       r_pts[0, 0] * S, i_pts[0, 0] * S))
        self.detours.sort(key=lambda d: d.x_fork)

    def detour(self, name: str) -> Detour:
        return next(d for d in self.detours if d.name == name)

    # --- routes --------------------------------------------------------------

    def main_route(self) -> Route:
        return Route(self.centre)

    def detour_route(self, det: Detour):
        """Open route: main from the start to the fork, the detour, main from
        the merge onward until the fork is reached again. Returns the route
        and the arclengths where it leaves and rejoins the main centreline."""
        main = self.centre
        yt_c = (self.y_top_outer + self.y_top_inner) / 2
        yb_c = (self.y_bot_outer + self.y_bot_inner) / 2
        i_fork = _nearest_index(main, (det.x_fork, yt_c))
        i_merge = _nearest_index(main, (det.x_fork, yb_c))
        lead = int(0.15 / _STEP_M)
        c = det.centre
        cs = _cumlen(c)
        j0 = int(np.searchsorted(cs, 0.5 * self.lane_m))
        j1 = int(np.searchsorted(cs, cs[-1] - 0.5 * self.lane_m))
        a = main[: i_fork - lead]
        h0 = _hermite(a[-1], a[-1] - a[-2], c[j0], c[j0 + 1] - c[j0])
        mid = c[j0 + 1: j1]
        b = main[i_merge + lead: i_fork]
        h1 = _hermite(mid[-1], mid[-1] - mid[-2], b[0], b[1] - b[0])
        pts = _resample(np.vstack([a, h0[1:], mid, h1[1:], b[1:]]), _STEP_M)
        route = Route(pts)
        s_fork = route.nearest_s(a[-1])
        s_merge = route.nearest_s(b[0])
        return route, s_fork, s_merge

    def detour_boundaries(self, det: Detour):
        yt_i, yb_i = self.y_top_inner, self.y_bot_inner
        inner, outer = self.inner, self.outer
        i_f = _nearest_index(inner, (det.x_fork, yt_i))
        i_m = _nearest_index(inner, (det.x_fork, yb_i))
        right = np.vstack([inner[:i_f], det.right_line, inner[i_m:]])
        o_f = _nearest_index(outer, (det.x_fork, self.y_top_outer))
        o_w = _nearest_index(outer, (det.x_wedge, self.y_bot_outer))
        left = np.vstack([outer[:o_f], [[det.x_fork, yt_i], [det.x_wedge, yt_i]], det.island_line,
                          [[det.x_wedge, self.y_bot_outer]], outer[o_w:]])
        return _resample(left, _STEP_M), _resample(right, _STEP_M)

    # --- phases ------------------------------------------------------------------

    def phase_of(self, pose) -> str:
        x, y = pose[0], pose[1]
        margin = 0.5 * self.lane_m
        for det in self.detours:
            if self.y_top_inner + margin < y < self.y_bot_inner - margin:
                e, _ = _signed_lateral(det.centre, (x, y))
                if abs(e) < 1.2 * self.lane_m:
                    return f"detour:{det.name}"
        e, i = _signed_lateral(self.centre, (x, y))
        px, py = self.centre[i]
        if px < self.cx_l or px > self.cx_r:
            return "U-turn"
        for det in self.detours:
            if py < self.cy:
                if det.x_fork - _SIGN_AHEAD_M - 0.3 <= px < det.x_fork:
                    return f"approach:{det.name}"
                if det.x_fork <= px <= det.x_wedge:
                    return f"entrance:{det.name}"
            else:
                if det.x_fork <= px <= det.x_wedge:
                    return f"merge:{det.name}"
                if det.x_fork - 1.0 <= px < det.x_fork:
                    return f"after-merge:{det.name}"
        return "straight"


# --- truth ---------------------------------------------------------------------

def truth_offset(route: Route, idx: int, pose, camera: CameraModel, params: Config) -> Optional[float]:
    """Where the route's own centreline sits in the frame, measured the way
    the tracker measures: the median over the scan rows of the centreline's
    column at each row's depth, minus the ROI centre, in downscaled pixels."""
    ratio = params.downscale_ratio
    height = round(camera.height / ratio)
    width = round(camera.width / ratio)
    roi_x0 = round(width * params.roi_x0)
    roi_x1 = round(width * params.roi_x1)
    roi_y0 = round(height * params.roi_y0)
    roi_y1 = round(height * params.roi_y1)
    roi_center_x = (roi_x0 + roi_x1) // 2
    ahead = route.pts[idx: idx + int(4.0 / _STEP_M) + 1]
    u, v, _w, valid = camera.project(ahead, pose, 0.0)
    u, v = u[valid], v[valid]
    if len(u) < 2:
        return None
    columns = []
    for sy in np.linspace(roi_y0, roi_y1, max(1, params.scan_lines), dtype=int):
        vt = sy * ratio
        below = np.flatnonzero(v <= vt)
        if not len(below) or below[0] == 0:
            continue
        i = below[0]
        t = (v[i - 1] - vt) / (v[i - 1] - v[i]) if v[i - 1] != v[i] else 0.0
        columns.append((u[i - 1] + t * (u[i] - u[i - 1])) / ratio)
    if not columns:
        return None
    return float(np.median(columns)) - roi_center_x


# --- metrics --------------------------------------------------------------------

@dataclass
class PhaseMetrics:
    n: int = 0
    seconds: float = 0.0
    lost: int = 0
    lost_run_s: float = 0.0
    hold_run_s: float = 0.0
    lat_px: list = field(default_factory=list)
    lat_m: list = field(default_factory=list)
    trk_err: list = field(default_factory=list)
    min_line_m: float = float("inf")
    crossings: int = 0
    sign_changes: int = 0
    reacq_deg: list = field(default_factory=list)
    states: Counter = field(default_factory=Counter)
    provisional: int = 0


@dataclass
class RunResult:
    scenario: str
    phases: dict
    frames: int
    seconds: float
    outcome: str
    events: list
    used_detour: bool
    lost_run_s: float
    hold_run_s: float
    crossings: int
    reacq_max_deg: float
    census: dict


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class _NoSeedPID(_PIDController):
    """Sweep only: a PID whose `_prev_error` cannot be seeded from outside, so
    the hold-exit seed and `seed_derivative()` become no-ops."""

    def __init__(self, *args, **kwargs):
        self._pe = 0.0
        super().__init__(*args, **kwargs)

    @property
    def _prev_error(self):
        return self._pe

    @_prev_error.setter
    def _prev_error(self, _value):
        pass

    def update(self, error, dt):
        out = super().update(error, dt)
        if dt > 0:
            self._pe = error
        return out

    def reset(self):
        super().reset()
        self._pe = 0.0


def run_closed_loop(scenario: str, camera: CameraModel, car: CarModel, track: Track, params: Config,
                    fps: float = 28.3, limit: int = 1500, writer=None, verbose=False, d_seed=True) -> RunResult:
    dt = 1.0 / fps
    if scenario not in SCENARIOS:
        raise SystemExit(f"unknown scenario {scenario}")
    target = None
    s_fork = s_merge = None
    if scenario in ("detour-easy", "detour-hard", "forgot-L1", "early-L1"):
        target = track.detour("easy" if scenario == "detour-easy" else "hard")
        route, s_fork, s_merge = track.detour_route(target)
        left_b, right_b = track.detour_boundaries(target)
        # end a little after the U-turn that follows the merge
        i_m = route.index_of_s(s_merge)
        s_end = route.length - 0.1
        for i in range(i_m, len(route.pts)):
            x, y = route.pts[i]
            if y < track.cy and x >= track.cx_l + _TAIL_AFTER_UTURN_M:
                s_end = min(s_end, float(route.s[i]))
                break
    else:
        route = track.main_route()
        left_b, right_b = track.outer, track.inner
        s_end = route.length - 0.05
        if scenario == "straight-both-forks":
            d = max(track.detours, key=lambda d: d.x_fork)
            s_end = route.nearest_s((d.x_wedge + 0.5, (track.y_top_outer + track.y_top_inner) / 2))

    base_path = Path.STRAIGHT if scenario in ("loop-straight", "straight-both-forks") else Path.NORMAL
    path = base_path
    tracker = LineTracker()
    auto = Autonomous()
    if not d_seed:
        auto._pid = _NoSeedPID(kp=0.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0))
    bike = Bicycle(car, *track.start)
    queue = deque([(0.0, 0.0)] * car.delay_frames)
    phases: dict = {}
    events = []
    pressed_r1 = pressed_l1 = False
    idx = 0
    progress = 0.0
    prev_s = 0.0
    lost_run = hold_run = 0.0
    lost_run_max = hold_run_max = 0.0
    prev_holding = 0.0
    prev_sign = 0
    inside_prev = True
    stopped_s = 0.0
    outcome = "limit"
    used_detour = False
    census = {"rows": Counter(), "both": Counter(), "off": [], "curv": []}
    res = None
    frame_index = 0
    t0 = time.time()
    while frame_index < limit:
        pose = bike.pose
        cam_pose = bike.camera_pose()
        idx, s, e, tangent = route.advance(idx, pose[:2])
        ds = s - prev_s
        if ds < -route.length / 2:
            ds += route.length
        progress += max(0.0, ds)
        prev_s = s
        phase = track.phase_of(pose)

        # scripted operator
        if target is not None:
            if not pressed_r1 and phase == f"approach:{target.name}":
                pressed_r1 = True
                path = Path.DETOUR
                auto.seed_derivative() if d_seed else None
                events.append((frame_index, phase, "R1 -> Detour"))
            elif pressed_r1 and not pressed_l1 and res is not None:
                home = False
                if scenario == "early-L1":
                    home = (progress - s_fork) / max(1e-6, s_merge - s_fork) >= 0.5
                elif scenario in ("detour-easy", "detour-hard"):
                    st = res.lane_state
                    home = (progress >= s_merge and st.startswith("anchor-right") and "fallback" not in st
                            and "provisional" not in st and res.rows_found >= 2)
                if home:
                    pressed_l1 = True
                    path = HOME_PATH
                    auto.seed_derivative() if d_seed else None
                    events.append((frame_index, phase, f"L1 -> {HOME_PATH.value}"))

        frame = camera.render(track.lines, cam_pose, track.model.tape_m)
        res = tracker.process(frame, params, anchor_for(path))
        profile = profile_for(path)
        steering, throttle = auto.step(res.lane_offset, res.lane_curvature, params, dt, profile=profile, provisional=res.provisional)
        used_detour |= auto.profile.value == "detour"
        queue.append((steering, throttle))
        cmd = queue.popleft()

        if writer is not None:
            writer.write(np.hstack([res.display_frame, res.blur_frame]))

        # metrics
        rec = phases.setdefault(phase, PhaseMetrics())
        rec.n += 1
        rec.seconds += dt
        rec.states[res.lane_state.split(" provisional")[0].split()[0] if res.lane_state else "?"] += 1
        rec.provisional += int(res.provisional)
        census["rows"][res.rows_found] += 1
        census["both"][res.both_rows] += 1
        if res.lane_offset is None:
            rec.lost += 1
            lost_run += dt
        else:
            lost_run = 0.0
            census["off"].append(res.lane_offset)
            if res.lane_curvature is not None:
                census["curv"].append(res.lane_curvature)
        lost_run_max = max(lost_run_max, lost_run)
        rec.lost_run_s = max(rec.lost_run_s, lost_run)
        hold_run = hold_run + dt if auto.holding > 0 else 0.0
        hold_run_max = max(hold_run_max, hold_run)
        rec.hold_run_s = max(rec.hold_run_s, hold_run)
        truth = truth_offset(route, idx, cam_pose, camera, params)
        if truth is not None:
            rec.lat_px.append(truth)
            if res.lane_offset is not None:
                rec.trk_err.append(res.lane_offset - truth)
        rec.lat_m.append(e)
        dl, _ = _signed_lateral(left_b, pose[:2])
        dr, _ = _signed_lateral(right_b, pose[:2])
        dr = -dr
        rec.min_line_m = min(rec.min_line_m, dl, dr)
        inside = dl > 0 and dr > 0
        if inside_prev and not inside:
            rec.crossings += 1
            events.append((frame_index, phase, f"line crossed ({'left' if dl <= 0 else 'right'})"))
        inside_prev = inside
        sign = 0 if abs(steering) < _STEER_DEADBAND else (1 if steering > 0 else -1)
        if sign and prev_sign and sign != prev_sign:
            rec.sign_changes += 1
        if sign:
            prev_sign = sign
        if prev_holding > 0 and auto.holding == 0 and res.lane_offset is not None:
            err = abs(math.degrees(_wrap(pose[2] - tangent)))
            rec.reacq_deg.append(err)
            events.append((frame_index, phase, f"re-acquired after {prev_holding:.2f} s hold, heading error {err:.0f} deg"))
        prev_holding = auto.holding

        if verbose and frame_index % 15 == 0:
            print(f"{frame_index:4d} s={progress:5.2f} {phase:18s} {path.value:8s} {res.lane_state:32s} off={res.lane_offset} "
                  f"curv={None if res.lane_curvature is None else round(res.lane_curvature, 3)} st={steering:+.2f} th={throttle:.2f} e={e:+.3f}")

        bike.step(cmd[0], cmd[1], dt)
        frame_index += 1

        stopped_s = stopped_s + dt if (auto.stop_reason == "lane lost") else 0.0
        if stopped_s > _STOP_BUDGET_S:
            outcome = "stopped (lane lost)"
            break
        if abs(e) > _OFF_ROAD_M:
            outcome = "off-road"
            break
        if progress >= s_end:
            outcome = "completed"
            break
    if verbose:
        print(f"  {frame_index} frames in {time.time() - t0:.1f} s")
    reacq = [d for rec in phases.values() for d in rec.reacq_deg]
    return RunResult(scenario, phases, frame_index, frame_index * dt, outcome, events, used_detour,
                     lost_run_max, hold_run_max, sum(r.crossings for r in phases.values()),
                     max(reacq) if reacq else 0.0, census)


# --- report / check -------------------------------------------------------------

def report(result: RunResult):
    print(f"scenario {result.scenario}: {result.frames} frames ({result.seconds:.1f} s), {result.outcome}; "
          f"longest LOST {result.lost_run_s:.2f} s, longest hold {result.hold_run_s:.2f} s, crossings {result.crossings}, "
          f"worst re-acq heading {result.reacq_max_deg:.0f} deg")
    print("  phase                n  lost  lostrun  hold   lat px mean/max   trk err mean/max   line m   cross  sgn/s  reacq  states")
    for phase, rec in result.phases.items():
        lat = np.array(rec.lat_px) if rec.lat_px else np.zeros(1)
        err = np.array(rec.trk_err) if rec.trk_err else np.zeros(1)
        rate = rec.sign_changes / rec.seconds if rec.seconds else 0.0
        reacq = max(rec.reacq_deg) if rec.reacq_deg else float("nan")
        states = ", ".join(f"{k} {v}" for k, v in sorted(rec.states.items(), key=lambda kv: -kv[1]))
        line_m = rec.min_line_m if rec.min_line_m != float("inf") else float("nan")
        print(f"  {phase:18s} {rec.n:5d} {rec.lost:5d}  {rec.lost_run_s:5.2f}  {rec.hold_run_s:5.2f}   {lat.mean():+7.1f} / {np.abs(lat).max():5.1f}"
              f"     {err.mean():+7.1f} / {np.abs(err).max():5.1f}     {line_m:+6.3f}   {rec.crossings:3d}   {rate:5.2f}  {reacq:5.0f}  {states}")
    for index, phase, what in result.events:
        print(f"  frame {index} ({phase}): {what}")


def _lat_px_mean(result: RunResult) -> float:
    vals = [abs(v) for rec in result.phases.values() for v in rec.lat_px]
    return float(np.mean(vals)) if vals else 0.0


def _sign_rate(result: RunResult) -> float:
    n = sum(r.sign_changes for r in result.phases.values())
    return n / result.seconds if result.seconds else 0.0


def check(results: dict) -> list:
    failures = []
    for name, r in results.items():
        if r.outcome != "completed":
            failures.append(f"{name}: run ended by '{r.outcome}'")
        if r.crossings:
            failures.append(f"{name}: {r.crossings} line crossing(s)")
        hold = autonomous_mod._DETOUR_HOLD_S if r.used_detour else autonomous_mod._LANE_LOSS_TIMEOUT
        if r.lost_run_s >= hold:
            failures.append(f"{name}: LOST run {r.lost_run_s:.2f} s reached the {hold:.1f} s hold")
        if r.reacq_max_deg >= _HEADING_REACQ_BUDGET_DEG:
            failures.append(f"{name}: heading error {r.reacq_max_deg:.0f} deg at re-acquisition over {_HEADING_REACQ_BUDGET_DEG:.0f}")
    if "loop-normal" in results and "loop-straight" in results:
        n, s = results["loop-normal"], results["loop-straight"]
        dpx = _lat_px_mean(s) - _lat_px_mean(n)
        if dpx > _STRAIGHT_VS_NORMAL_PX:
            failures.append(f"loop-straight: lateral error {dpx:+.1f} px worse than loop-normal (budget {_STRAIGHT_VS_NORMAL_PX:.0f})")
        rn, rs = _sign_rate(n), _sign_rate(s)
        if rs > rn * (1 + _STRAIGHT_VS_NORMAL_RATE) + _STRAIGHT_VS_NORMAL_RATE_FLOOR:
            failures.append(f"loop-straight: steering sign-change rate {rs:.2f}/s vs loop-normal {rn:.2f}/s (budget +{_STRAIGHT_VS_NORMAL_RATE:.0%})")
    return failures


# --- --fit-widths ----------------------------------------------------------------

def measure_widths(camera: CameraModel, track: Track, params: Config):
    """(row, width) in downscaled px of the lane on a straight, lane centre,
    from the band top to the bottom of the ROI, plus the frame."""
    ratio = params.downscale_ratio
    bike = Bicycle(CarModel(), *track.start)
    frame = camera.render(track.lines, bike.camera_pose(), track.model.tape_m)
    height = round(camera.height / ratio)
    width = round(camera.width / ratio)
    small = cv.resize(frame, (width, height))
    gray = cv.cvtColor(small, cv.COLOR_BGR2GRAY)
    binary = gray > 128
    c = width // 2
    rows = []
    for y in range(round(0.35 * height), round(height * params.roi_y1) + 1, 4):
        lefts = np.flatnonzero(binary[y, :c])
        rights = np.flatnonzero(binary[y, c:])
        if len(lefts) and len(rights):
            rows.append((y, int(rights[0]) + c - int(lefts[-1])))
        else:
            rows.append((y, None))
    return rows, small, height, width


def fit_widths(camera: CameraModel, track: Track, params: Config, recorded: Optional[tuple]) -> bool:
    rows, _small, height, width = measure_widths(camera, track, params)
    seen = [(y, w) for y, w in rows if w is not None]
    print(f"camera {camera.name}: lane width vs row (downscaled px), frame {width}x{height}")
    for y, w in rows:
        print(f"  row {y:4d} ({y / height:.3f} h): {'no line' if w is None else f'{w} px ({w / width:.0%} of width)'}")
    ok = True
    y67 = round(0.67 * height)
    w67 = next((w for y, w in seen if abs(y - y67) <= 2), None)
    near = [w for y, w in rows if y >= 0.87 * height]
    print(f"  span at 0.67 h: {'none' if w67 is None else f'{w67 / width:.0%}'}; rows >= 0.87 h see a line: {any(w is not None for w in near)}")
    if len(seen) >= 2:
        a, b = np.polyfit([y for y, _ in seen], [w for _, w in seen], 1)
        print(f"  sim linear profile: width = {a:.4f} * y + {b:.2f}")
    if recorded is not None:
        a, b = recorded
        errs = [(y, w, abs(w - (a * y + b)) / max(1.0, a * y + b)) for y, w in seen]
        worst = max(errs, key=lambda t: t[2]) if errs else None
        if worst is None:
            print("  no rows with both lines; cannot compare")
            ok = False
        else:
            ok = worst[2] <= _FIT_WIDTH_BUDGET
            print(f"  vs recorded profile {a},{b}: max relative error {worst[2]:.1%} at row {worst[0]} "
                  f"(sim {worst[1]} px, recorded {a * worst[0] + b:.0f} px) -> {'PASS' if ok else 'FAIL'} (budget {_FIT_WIDTH_BUDGET:.0%})")
    return ok


# --- --replay-census -------------------------------------------------------------

def census_of_recording(path: str, params: Config, limit: int):
    cap = cv.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {path}")
    tracker = LineTracker()
    c = {"rows": Counter(), "both": Counter(), "off": [], "curv": []}
    n = 0
    while n < limit:
        ok, frame = cap.read()
        if not ok:
            break
        res = tracker.process(frame, params, LaneAnchor.AUTO)
        c["rows"][res.rows_found] += 1
        c["both"][res.both_rows] += 1
        if res.lane_offset is not None:
            c["off"].append(res.lane_offset)
            if res.lane_curvature is not None:
                c["curv"].append(res.lane_curvature)
        n += 1
    cap.release()
    return c, n


def _hist(counter: Counter, bins: int):
    total = sum(counter.values()) or 1
    return [counter.get(i, 0) / total for i in range(bins + 1)]


def _q(vals, q):
    return float(np.percentile(vals, q)) if len(vals) else float("nan")


def replay_census(recording: str, camera: CameraModel, car: CarModel, track: Track, params: Config, fps: float, limit: int) -> bool:
    real, n_real = census_of_recording(recording, params, limit)
    sim = run_closed_loop("loop-normal", camera, car, track, params, fps=fps, limit=limit).census
    bins = params.scan_lines
    print(f"census: recording {recording} ({n_real} frames) vs sim loop-normal ({camera.name} camera, {sum(sim['rows'].values())} frames)")
    ok = True
    for key in ("rows", "both"):
        hr, hs = _hist(real[key], bins), _hist(sim[key], bins)
        print(f"  {'rows_found' if key == 'rows' else 'both_rows '}   " + "  ".join(f"{i}: {a:5.1%} | {b:5.1%}" for i, (a, b) in enumerate(zip(hr, hs))))
        worst = max(abs(a - b) for a, b in zip(hr, hs))
        ok &= worst <= _CENSUS_ROWS_BUDGET
        print(f"               worst bin gap {worst:.1%} (budget {_CENSUS_ROWS_BUDGET:.0%})")
    for key, label, budget in (("off", "offset px", _CENSUS_OFFSET_BUDGET_PX), ("curv", "curvature", _CENSUS_CURVATURE_BUDGET)):
        print(f"  {label:10s} p10/p50/p90  real {_q(real[key], 10):+7.2f} {_q(real[key], 50):+7.2f} {_q(real[key], 90):+7.2f}"
              f"   sim {_q(sim[key], 10):+7.2f} {_q(sim[key], 50):+7.2f} {_q(sim[key], 90):+7.2f}")
        gap = abs(_q(real[key], 50) - _q(sim[key], 50))
        ok &= gap <= budget
        print(f"               p50 gap {gap:.2f} (budget {budget})")
    print("census " + ("MATCH: constants may be read off the sim" if ok else "MISMATCH: do not read constants off the sim"))
    return ok


# --- --sweep ----------------------------------------------------------------------

def sweep(scenarios, camera: CameraModel, car: CarModel, model: TrackModel, params: Config, fps: float, limit: int):
    hold0, slew0 = autonomous_mod._DETOUR_HOLD_S, autonomous_mod._DETOUR_STEER_RATE
    variants = [
        ("baseline", {}),
        ("R_lock -20%", {"car": replace(car, r_lock_m=car.r_lock_m * 0.8)}),
        ("R_lock +20%", {"car": replace(car, r_lock_m=car.r_lock_m * 1.2)}),
        ("speed -20%", {"car": replace(car, speed_table=tuple((t, v * 0.8) for t, v in car.speed_table))}),
        ("speed +20%", {"car": replace(car, speed_table=tuple((t, v * 1.2) for t, v in car.speed_table))}),
        ("pitch -5 deg", {"camera": replace(camera, pitch_deg=camera.pitch_deg - 5)}),
        ("pitch +5 deg", {"camera": replace(camera, pitch_deg=camera.pitch_deg + 5)}),
        ("width 0.9", {"track": replace(model, width_factor=0.9)}),
        ("width 1.1", {"track": replace(model, width_factor=1.1)}),
        ("hold 0.2 s", {"hold": 0.2}),
        ("hold 8 s", {"hold": 8.0}),
        ("slew off", {"slew": 1e9}),
        ("D-seed off", {"d_seed": False}),
        ("Kp 0.0143", {"params": replace(params, kp=0.0143)}),
        ("Kp 0.0231", {"params": replace(params, kp=0.0231)}),
    ]
    rows = []
    for name, v in variants:
        autonomous_mod._DETOUR_HOLD_S = v.get("hold", hold0)
        autonomous_mod._DETOUR_STEER_RATE = v.get("slew", slew0)
        trk = Track(v.get("track", model))
        cells = []
        fails = 0
        cross = 0
        lost = 0.0
        try:
            for sc in scenarios:
                r = run_closed_loop(sc, v.get("camera", camera), v.get("car", car), trk, v.get("params", params),
                                    fps=fps, limit=limit, d_seed=v.get("d_seed", True))
                f = check({sc: r})
                fails += bool(f)
                cross += r.crossings
                lost = max(lost, r.lost_run_s)
                cells.append(f"{'PASS' if not f else 'FAIL':4s} x{r.crossings} lost{r.lost_run_s:4.1f}s re{r.reacq_max_deg:3.0f} {r.outcome[:9]:9s}")
        finally:
            autonomous_mod._DETOUR_HOLD_S = hold0
            autonomous_mod._DETOUR_STEER_RATE = slew0
        rows.append((fails, cross, lost, name, cells))
        print(f"  {name:14s} " + " | ".join(cells), flush=True)
    print("\nranking (fewest failing scenarios, then crossings, then longest LOST):")
    print("  variant         " + " | ".join(f"{s:38s}" for s in scenarios))
    for fails, cross, lost, name, cells in sorted(rows, key=lambda t: (t[0], t[1], t[2])):
        print(f"  {name:14s} " + " | ".join(cells))


# --- params ---------------------------------------------------------------------

def print_params(camera: CameraModel, car: CarModel, model: TrackModel, fps: float):
    print("camera", camera.name, "(PLACEHOLDER, measure)" if camera.placeholder else "(manila tool values)")
    for k, v in camera.__dict__.items():
        print(f"  {k:14s} {v}")
    print("car (all PLACEHOLDER: measure on the car)")
    for k, v in car.__dict__.items():
        print(f"  {k:14s} {v}")
    print(f"  {'max_steer_deg':14s} {math.degrees(car.max_steer):.1f}  (from wheelbase / R_lock)")
    print("track (SVG constants from the 3d-simulator; PLACEHOLDER: taped_lane_m, tape_m, fork_fillet_m, merge_angle_deg)")
    for k, v in model.__dict__.items():
        if k != "detours":
            print(f"  {k:14s} {v}")
    print(f"  {'scale m/unit':14s} {model.scale:.6f}")
    print(f"  fps {fps}, command delay {car.delay_frames} frames = {car.delay_frames / fps:.3f} s")


# --- main --------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default=",".join(SCENARIOS), help="comma-separated list")
    parser.add_argument("--camera", choices=sorted(CAMERAS), default="manila")
    parser.add_argument("--fps", type=float, default=28.3)
    parser.add_argument("--limit", type=int, default=1500, help="max frames per scenario")
    parser.add_argument("--out", help="write [frame | tracker debug] of a single scenario to this mp4")
    parser.add_argument("--check", action="store_true", help="exit 1 on a regression (see module docstring)")
    parser.add_argument("--sweep", action="store_true", help="robustness ranking over parameter variants")
    parser.add_argument("--replay-census", metavar="RECORDING", help="compare tracker census on a recording vs the sim")
    parser.add_argument("--fit-widths", metavar="SLOPE,INTERCEPT", help="compare the sim's lane width vs row against a recorded profile")
    parser.add_argument("--print-params", action="store_true")
    parser.add_argument("--width-factor", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    camera = CAMERAS[args.camera]
    car = CarModel()
    model = TrackModel(width_factor=args.width_factor)
    params = Config()
    scenarios = [s for s in args.scenario.split(",") if s]

    if args.print_params:
        print_params(camera, car, model, args.fps)
        return 0
    track = Track(model)
    if args.fit_widths:
        recorded = tuple(float(v) for v in args.fit_widths.split(",")) if "," in args.fit_widths else None
        return 0 if fit_widths(camera, track, params, recorded) else 1
    if args.replay_census:
        return 0 if replay_census(args.replay_census, camera, car, track, params, args.fps, args.limit) else 1
    if args.sweep:
        default = [s for s in ("detour-easy", "detour-hard") if s in scenarios] or scenarios
        if args.scenario == ",".join(SCENARIOS):
            default = ["detour-easy", "detour-hard"]
        sweep(default, camera, car, model, params, args.fps, args.limit)
        return 0

    results = {}
    for sc in scenarios:
        writer = None
        if args.out and len(scenarios) == 1:
            w = round(camera.width / params.downscale_ratio)
            h = round(camera.height / params.downscale_ratio)
            writer = cv.VideoWriter(args.out, cv.VideoWriter_fourcc(*"mp4v"), args.fps, (2 * w, h))
        r = run_closed_loop(sc, camera, car, track, params, fps=args.fps, limit=args.limit, writer=writer, verbose=args.verbose)
        if writer is not None:
            writer.release()
        report(r)
        results[sc] = r
    if args.check:
        failures = check(results)
        for f in failures:
            print(f"FAIL: {f}")
        print("PASS" if not failures else "FAIL")
        return 1 if failures else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
