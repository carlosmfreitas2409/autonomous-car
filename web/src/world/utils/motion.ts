import type { CartState } from '@/types';

import { LANE_REAL_M } from './units';

const SPEED_TABLE: [number, number][] = [
  [0, 0],
  [0.375, 0.45],
  [0.5, 0.6],
  [0.63, 0.8],
];

const TRAIL_STEP = 0.05;

interface TrailPose {
  x: number;
  z: number;
  angle: number;
  back: number;
}

function tableSpeed(output: number) {
  const a = Math.abs(output);
  for (let i = 1; i < SPEED_TABLE.length; i++) {
    const [t0, v0] = SPEED_TABLE[i - 1];
    const [t1, v1] = SPEED_TABLE[i];
    if (a <= t1) return v0 + ((a - t0) * (v1 - v0)) / (t1 - t0);
  }
  const [t0, v0] = SPEED_TABLE.at(-2)!;
  const [t1, v1] = SPEED_TABLE.at(-1)!;
  return v1 + ((a - t1) * (v1 - v0)) / (t1 - t0);
}

export function groundSpeed(state: CartState | null) {
  if (!state || state.drive.estopped) return 0;
  const { speed_est_mps: estimate, applied_throttle: output } = state.drive;
  const mps = estimate ?? tableSpeed(output);
  return (Math.sign(output) * mps) / LANE_REAL_M;
}

export function carried(x: number, z: number, ds: number, turn: number) {
  const c = Math.cos(turn);
  const s = Math.sin(turn);
  const back = z - ds;
  return { x: x * c - back * s, z: x * s + back * c };
}

export class Trail {
  private poses: TrailPose[] = [];

  advance(ds: number, turn: number, keep: number) {
    for (const pose of this.poses) {
      const moved = carried(pose.x, pose.z, ds, turn);
      pose.x = moved.x;
      pose.z = moved.z;
      pose.angle -= turn;
      pose.back += ds;
    }
    if (!this.poses.length || this.poses[0].back >= TRAIL_STEP) {
      this.poses.unshift({ x: 0, z: 0, angle: 0, back: 0 });
    }
    while (this.poses.length > 2 && this.poses.at(-2)!.back >= keep) this.poses.pop();
  }

  clear() {
    this.poses = [];
  }

  at(s: number) {
    const want = Math.max(0, -s);
    let a: TrailPose = { x: 0, z: 0, angle: 0, back: 0 };
    for (const b of this.poses) {
      if (b.back >= want) {
        const f = b.back > a.back ? (want - a.back) / (b.back - a.back) : 0;
        return {
          x: a.x + (b.x - a.x) * f,
          z: a.z + (b.z - a.z) * f,
          angle: a.angle + (b.angle - a.angle) * f,
        };
      }
      a = b;
    }
    return null;
  }
}
