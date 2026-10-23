import type * as THREE from 'three';

import {
  BRANCH_CURVATURE,
  BRANCH_TURN,
  type ForkView,
  NOSE_ALONG_BRANCH,
  NOSE_ALONG_MAIN,
} from './fork-watch';
import type { Trail } from './motion';
import { type Pose, type RoadCurve, type RoadShape, across, arcPoint } from './road';
import type { Strip, StripPoint } from './strips';
import { LANE_BEHIND_M, LANE_FADE_M, LANE_SEEN_M, LANE_WIDTH_M } from './units';

const BOARD_M = 0.64;
const BOARD_ALPHA = 0.07;
const MARK_HALF = 0.023;
const MARK_ALPHA = 0.72;
const VERGE: [number, number, number][] = [
  [0.56, 0.07, 0.04],
  [0.63, 0.04, 0.05],
  [0.72, 0.02, 0.07],
];
const SAMPLE_STEP = 0.1;

type Colour = 'road' | 'kerb' | 'lost' | 'marking' | 'stop';

interface RoadInput {
  lane: RoadShape;
  curve: RoadCurve;
  trail: Trail;
  fork: ForkView | null;
  mainAlpha: number;
  travelled: number;
  isLive: boolean;
  isLost: boolean;
  colours: Record<Colour, THREE.Color>;
}

function fading(seen: number, end: number) {
  return (s: number) => {
    if (s < -LANE_BEHIND_M) return 0;
    if (s < -(LANE_BEHIND_M - 0.8)) return (s + LANE_BEHIND_M) / 0.8;
    if (s < seen) return 1;
    if (s < end) return 1 - (s - seen) / (end - seen);
    return 0;
  };
}

function beside(pose: Pose, side: number, a: number): StripPoint {
  return { ...across(pose, side * (LANE_WIDTH_M / 2)), a };
}

function sample(point: (s: number) => StripPoint, s0: number, s1: number) {
  const points: StripPoint[] = [];
  for (let s = s0; s < s1; s += SAMPLE_STEP) points.push(point(s));
  points.push(point(s1));
  return points;
}

function within(s: number, a: number, b: number) {
  return Math.max(0, Math.min(1, (s - a) / 0.15 + 0.5, (b - s) / 0.15 + 0.5));
}

export function roadLayer(input: RoadInput) {
  const { lane, curve, trail, fork, mainAlpha, travelled, isLive, isLost, colours } = input;
  const end = LANE_FADE_M;
  const fade = fading(LANE_SEEN_M, end);
  const dim = isLive ? 1 : 0.55;

  const heading = Math.atan(lane.heading);
  const cosHeading = Math.cos(heading);
  const sinHeading = Math.sin(heading);
  const roadPoint = (s: number): Pose => {
    if (s >= 0) return curve(s);
    const behind = trail.at(s);
    if (!behind) return curve(s);
    return {
      x: lane.centre + behind.x * cosHeading - behind.z * sinHeading,
      z: behind.x * sinHeading + behind.z * cosHeading,
      angle: behind.angle + heading,
    };
  };
  const onLane =
    (side: number, alpha: (s: number) => number = () => 1) =>
    (s: number) =>
      beside(roadPoint(s), side, fade(s) * alpha(s) * dim);

  const roads: Strip[] = [
    {
      points: sample(
        onLane(0, () => (isLost ? 0.6 : 1)),
        -LANE_BEHIND_M,
        end,
      ),
      half: LANE_WIDTH_M / 2,
      color: isLost ? colours.lost : colours.road,
    },
  ];
  for (const side of [-1, 1]) {
    for (const [offset, alpha, half] of VERGE) {
      const shade = (s: number): StripPoint => {
        const point = roadPoint(s);
        const o = side * offset * LANE_WIDTH_M;
        return {
          x: point.x + o * Math.cos(point.angle),
          z: point.z - o * Math.sin(point.angle),
          a: fade(s) * alpha * dim,
        };
      };
      roads.push({ points: sample(shade, -LANE_BEHIND_M, end), half, color: colours.kerb });
    }
  }

  const marks: Strip[] = [];
  const tape = isLost ? colours.stop : colours.marking;
  const line = (point: (s: number) => StripPoint, s0: number, s1: number, colour = tape) => {
    const dimmed = (colour === tape && isLost ? 0.7 : 1) * MARK_ALPHA;
    const faded = (s: number) => {
      const p = point(s);
      return { ...p, a: p.a * dimmed };
    };
    marks.push({ points: sample(faded, s0, s1), half: MARK_HALF, color: colour });
  };
  const boards = (
    point: (s: number) => Pose,
    at: (s: number) => number,
    s0: number,
    s1: number,
  ) => {
    const phase = (((s0 + travelled) % BOARD_M) + BOARD_M) % BOARD_M;
    for (let s = s0 - phase; s < s1; s += BOARD_M) {
      if (s < s0) continue;
      const a = at(s) * BOARD_ALPHA;
      if (a < 0.004) continue;
      const p = point(s);
      marks.push({
        points: [beside(p, -0.92, a), beside(p, 0.92, a)],
        half: 0.022,
        color: colours.kerb,
      });
    }
  };

  const branching =
    fork && !fork.isTurning && fork.at > -LANE_BEHIND_M && fork.at < end ? fork : null;
  const leaving = fork && fork.isTurning && mainAlpha > 0.02 ? fork : null;

  for (const side of [-1, 1]) {
    const mouth = side > 0 && branching ? branching : side < 0 && leaving ? leaving : null;
    const open = mouth
      ? (s: number) =>
          1 -
          mouth.alpha * (side < 0 ? mainAlpha : 1) * within(s, mouth.at, mouth.at + NOSE_ALONG_MAIN)
      : undefined;
    line(onLane(side, open), -LANE_BEHIND_M, end);
  }
  boards(roadPoint, (s) => fade(s) * dim, -LANE_BEHIND_M, end);

  if (branching) {
    const start = curve(branching.at);
    const length = BRANCH_TURN / BRANCH_CURVATURE;
    const shown = (s: number) =>
      branching.alpha * fade(branching.at + s) * Math.min(1, (length - s) / (0.4 * length));
    const onBranch = (side: number, emerges: boolean) => (s: number) =>
      beside(
        arcPoint(start.x, start.z, start.angle, BRANCH_CURVATURE, s),
        side,
        shown(s) * (emerges ? Math.min(1, s / 0.5) : 1),
      );
    roads.push({
      points: sample(onBranch(0, true), 0, length),
      half: LANE_WIDTH_M / 2,
      color: colours.road,
    });
    line(onBranch(1, false), 0, length);
    line(onBranch(-1, false), NOSE_ALONG_BRANCH, length);
  }

  if (leaving) {
    const turned = Math.min(BRANCH_TURN, -BRANCH_CURVATURE * Math.min(0, leaving.at));
    const mouth = {
      x: (1 - Math.cos(turned)) / BRANCH_CURVATURE,
      z: -Math.sin(turned) / BRANCH_CURVATURE,
    };
    const fadeMain = fading(LANE_SEEN_M, LANE_FADE_M);
    const shown = (s: number) => leaving.alpha * mainAlpha * fadeMain(s);
    const mainPoint = (s: number) => arcPoint(mouth.x, mouth.z, -turned, 0, s);
    const onMain = (side: number) => (s: number) => beside(mainPoint(s), side, shown(s));
    roads.push({
      points: sample(onMain(0), -LANE_BEHIND_M, LANE_FADE_M),
      half: LANE_WIDTH_M / 2,
      color: colours.road,
    });
    for (const side of [-1, 1]) line(onMain(side), -LANE_BEHIND_M, LANE_FADE_M, colours.marking);
    boards(mainPoint, shown, -LANE_BEHIND_M, LANE_FADE_M);
  }

  return { roads, marks };
}
