import { clamp01, smooth } from './math';

export interface Pose {
  x: number;
  z: number;
  angle: number;
}

export interface RoadShape {
  centre: number;
  heading: number;
  curvature: number;
}

export type RoadCurve = (s: number) => Pose;

export const BEND_IN = 0.5;
const MAX_TURN = 1.3;
const TURN_FREE = 0.6;
const CURVE_STEP = 0.05;

const PATH_LOOKAHEAD_S = 2.2;
const PATH_MIN = 0.45;

export function arcPoint(x: number, z: number, start: number, curvature: number, s: number): Pose {
  const angle = start + curvature * s;
  if (Math.abs(curvature) < 1e-4) {
    return { x: x + s * Math.sin(start), z: z + s * Math.cos(start), angle };
  }
  return {
    x: x + (Math.cos(start) - Math.cos(angle)) / curvature,
    z: z + (Math.sin(angle) - Math.sin(start)) / curvature,
    angle,
  };
}

export function lanePoint(x0: number, heading: number, curvature: number, s: number) {
  return arcPoint(x0, 0, Math.atan(heading), curvature, s);
}

export function across(pose: Pose, lateral: number) {
  return {
    x: pose.x + lateral * Math.cos(pose.angle),
    z: pose.z - lateral * Math.sin(pose.angle),
  };
}

export function pathReach(speed: number, wanted: number) {
  const reach = Math.min(wanted, Math.abs(speed) * PATH_LOOKAHEAD_S);
  return reach < PATH_MIN ? null : reach;
}

export function mergePath(
  road: RoadCurve,
  offset: number,
  from: number,
  to: number,
  merge: number,
  steps: number,
) {
  const points: { x: number; z: number; t: number }[] = [];
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const s = from + t * Math.max(0, to - from);
    const left = offset * (1 - smooth(clamp01(s / merge)));
    points.push({ ...across(road(s), left), t });
  }
  return points;
}

export function roadCurve(road: RoadShape, reach: number): RoadCurve {
  const start = Math.atan(road.heading);
  let x = road.centre;
  let z = 0;
  let angle = start;
  const table: Pose[] = [{ x, z, angle }];

  for (let i = 0; i * CURVE_STEP < Math.max(reach, CURVE_STEP); i++) {
    const mid = (i + 0.5) * CURVE_STEP;
    const eased = road.curvature * smooth(Math.min(1, mid / BEND_IN));
    const slack = clamp01(Math.max(0, 1 - Math.abs(angle - start) / MAX_TURN) / (1 - TURN_FREE));
    const turn = eased * slack * slack * CURVE_STEP;
    x += Math.sin(angle + turn / 2) * CURVE_STEP;
    z += Math.cos(angle + turn / 2) * CURVE_STEP;
    angle += turn;
    table.push({ x, z, angle });
  }

  return (s) => {
    if (s <= 0) {
      return { x: road.centre + s * Math.sin(start), z: s * Math.cos(start), angle: start };
    }
    const i = Math.min(table.length - 2, Math.floor(s / CURVE_STEP));
    const f = Math.min(1, s / CURVE_STEP - i);
    const a = table[i];
    const b = table[i + 1];
    return {
      x: a.x + (b.x - a.x) * f,
      z: a.z + (b.z - a.z) * f,
      angle: a.angle + (b.angle - a.angle) * f,
    };
  };
}
