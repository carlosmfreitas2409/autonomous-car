import type { Lane } from '@/types';

import { clamp } from './math';

const HALF_LANE_FRAC = 0.45;

export const CAMERA_HEIGHT_LANES = 0.45;

const CURVE_GAIN = 0.35;
const CURVE_MAX = 0.9;

export type Quadratic = [number, number, number];

export interface FloorPoint {
  d: number;
  x: number;
}

export interface LaneShape {
  curvature: number;
}

export function laneCentre(lane: Lane) {
  const [width] = lane.frame;
  return lane.offset_px === null ? 0 : lane.offset_px / (2 * width * HALF_LANE_FRAC);
}

export function floorMapping(lane: Lane) {
  const profile = lane.width_profile;
  if (!profile) return null;

  const [width, height] = lane.frame;
  const [a, b] = profile;
  const cos = Math.min(0.99, a * CAMERA_HEIGHT_LANES);
  if (cos <= 0.05) return null;
  const sin = Math.sqrt(1 - cos * cos);
  const focal = (b * CAMERA_HEIGHT_LANES + (height / 2) * cos) / sin;
  const cx = lane.roi_center_x ?? width / 2;

  return (x: number, y: number): FloorPoint | null => {
    const w = a * y + b;
    if (w <= 20) return null;
    return { d: (focal / w - CAMERA_HEIGHT_LANES * sin) / cos, x: (x - cx) / w };
  };
}

export function rowPoints(lane: Lane) {
  const toFloor = floorMapping(lane);
  if (!toFloor) return [];
  return lane.rows
    .map(([y, x]) => toFloor(x, y))
    .filter((point): point is FloorPoint => point !== null);
}

export function floorPoints(lane: Lane) {
  const toFloor = floorMapping(lane);
  if (!toFloor) return [];

  const [width] = lane.frame;
  const points: FloorPoint[] = [];
  for (const [y, left, right] of lane.band ?? []) {
    if (left <= 2 || right <= 2 || right >= width - 3) continue;
    const point = toFloor((left + right) / 2, y);
    if (point) points.push(point);
  }
  return [...points, ...rowPoints(lane)];
}

export function evaluate(q: Quadratic, d: number) {
  return q[0] + q[1] * d + q[2] * d * d;
}

function fitQuadratic(points: FloorPoint[]): Quadratic | null {
  const s = [0, 0, 0, 0, 0];
  const t = [0, 0, 0];
  for (const { d, x } of points) {
    let power = 1;
    for (let k = 0; k < 5; k++) {
      s[k] += power;
      if (k < 3) t[k] += x * power;
      power *= d;
    }
  }

  const m = [
    [s[0], s[1], s[2]],
    [s[1], s[2], s[3]],
    [s[2], s[3], s[4]],
  ];
  const det = (q: number[][]) =>
    q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1]) -
    q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0]) +
    q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0]);
  const base = det(m);
  if (Math.abs(base) < 1e-12) return null;

  const solve = (column: number) =>
    det(m.map((row, i) => row.map((value, j) => (j === column ? t[i] : value)))) / base;
  return [solve(0), solve(1), solve(2)];
}

export function robustQuadratic(points: FloorPoint[], min: number, floor: number) {
  if (points.length < min) return null;
  const depths = points.map((point) => point.d);
  if (Math.max(...depths) - Math.min(...depths) < 0.2) return null;

  let kept = points;
  let fit = fitQuadratic(kept);
  for (let round = 0; round < 2 && fit; round++) {
    const current = fit;
    const residual = (point: FloorPoint) => Math.abs(evaluate(current, point.d) - point.x);
    const typical = kept.map(residual).sort((a, b) => a - b)[Math.floor(kept.length / 2)];
    const next = points.filter((point) => residual(point) < Math.max(floor, 2.5 * typical));
    if (next.length < min) return null;
    kept = next;
    fit = fitQuadratic(kept);
  }
  return fit;
}

export function shapeOf(points: FloorPoint[]): LaneShape | null {
  const fit = robustQuadratic(points, 5, 0.04);
  if (!fit) return null;
  const [, c1, c2] = fit;
  const curvature = (2 * c2) / (1 + c1 * c1) ** 1.5;
  return { curvature: clamp(CURVE_GAIN * curvature, CURVE_MAX) };
}

export function laneShape(lane: Lane) {
  return shapeOf(floorPoints(lane));
}
