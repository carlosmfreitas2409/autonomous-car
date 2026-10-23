import type * as THREE from 'three';

import type { CartState } from '@/types';

import { BRANCH_CURVATURE, type ForkView } from './fork-watch';
import { type Pose, type RoadCurve, arcPoint, lanePoint, mergePath, pathReach } from './road';
import type { Strip, StripPoint } from './strips';
import { LOCK_RADIUS } from './units';

const PATH_START = 0.25;
const PATH_REACH = 2.6;
const MANUAL_REACH = 2.2;
const PATH_HALF = 0.15;
const PATH_HALF_END = 0.125;
const PATH_MERGE = 2.0;
const STEPS = 48;
const LIFT = 0.03;

interface PathInput {
  state: CartState | null;
  isLaneKnown: boolean;
  laneCentre: number;
  curve: RoadCurve;
  fork: ForkView | null;
  speed: number;
  steering: number;
  colours: { accent: THREE.Color; manual: THREE.Color };
}

function pathAlpha(t: number) {
  return 0.9 * Math.min(1, (1 - t) / 0.12);
}

function taper(t: number) {
  return PATH_HALF + (PATH_HALF_END - PATH_HALF) * t * t;
}

export function pathLayer(input: PathInput): Strip[] {
  const { state, colours } = input;
  if (!state || state.drive.estopped) return [];

  const followsLane =
    state.drive.mode === 'autonomous' && input.isLaneKnown && state.path.holding_s <= 0;
  if (followsLane) {
    const points = lanePath(input);
    return points ? [{ points, half: PATH_HALF, color: colours.accent }] : [];
  }

  const curvature = Math.max(-1, Math.min(1, input.steering)) / LOCK_RADIUS;
  const points: StripPoint[] = [];
  for (let i = 0; i <= STEPS; i++) {
    const t = i / STEPS;
    const point = lanePoint(0, 0, curvature, PATH_START + t * (MANUAL_REACH - PATH_START));
    points.push({ x: point.x, z: point.z + LIFT, a: pathAlpha(t) * 0.8, half: taper(t) * 0.85 });
  }
  const isManual = state.drive.mode === 'manual';
  return [{ points, half: PATH_HALF, color: isManual ? colours.manual : colours.accent }];
}

function lanePath({ curve, fork, speed, laneCentre }: PathInput) {
  const branch = fork && !fork.isTurning && fork.take === 'detour' && fork.at > -0.4 ? fork : null;
  const start = branch ? curve(branch.at) : null;
  const wanted = branch ? Math.min(3.2, Math.max(0, branch.at) + 1.8) : PATH_REACH;
  const reach = pathReach(speed, wanted);
  if (reach === null) return null;

  const ahead = (s: number): Pose => {
    const point = curve(s);
    if (!branch || !start || s <= branch.at) return point;
    const turn = arcPoint(start.x, start.z, start.angle, BRANCH_CURVATURE, s - branch.at);
    return {
      x: point.x + (turn.x - point.x) * branch.alpha,
      z: point.z + (turn.z - point.z) * branch.alpha,
      angle: point.angle + (turn.angle - point.angle) * branch.alpha,
    };
  };

  return mergePath(ahead, -laneCentre, PATH_START, reach, PATH_MERGE, STEPS).map(
    ({ x, z, t }): StripPoint => ({ x, z: z + LIFT, a: pathAlpha(t), half: taper(t) }),
  );
}
