import type { Lane } from '@/types';

import {
  type FloorPoint,
  type LaneShape,
  type Quadratic,
  evaluate,
  floorMapping,
  floorPoints,
  robustQuadratic,
  rowPoints,
  shapeOf,
} from './lane-fit';

const LEARN_RATE = 0.05;
const STRAIGHT_PAIR = 0.4;
const AGREE_GAP = 0.05;
const AGREE_BEND = 1.0;
const STRAIGHT = 0.6;
const OPEN_GAP = 0.08;
const OPEN_ROWS = 0.4;
const OUT_OF_PLACE = 0.06;

const OFFSET_SMOOTH = 0.25;
const MIN_RUN = 0.08;
const CROSS_MAX = 0.3;

export interface EdgeRow {
  d: number;
  left: number | null;
  right: number | null;
}

export interface LaneReading {
  shape: LaneShape | null;
  opening: number | null;
  near: number | null;
}

function bend(q: Quadratic) {
  return 2 * q[2];
}

function lineFit(points: FloorPoint[]) {
  return robustQuadratic(points, 6, 0.01);
}

export function bandEdges(lane: Lane) {
  const toFloor = floorMapping(lane);
  if (!toFloor) return [];

  const [width] = lane.frame;
  const rows: EdgeRow[] = [];
  for (const [y, left, right] of lane.band ?? []) {
    const l = left > 2 ? toFloor(left, y) : null;
    const r = right > 2 && right < width - 3 ? toFloor(right, y) : null;
    const d = l?.d ?? r?.d;
    if (d === undefined) continue;
    rows.push({ d, left: l?.x ?? null, right: r?.x ?? null });
  }
  return rows.sort((p, q) => p.d - q.d);
}

export class LaneHeading {
  private smoothed: number | null = null;
  private mark: number | null = null;
  private run = 0;
  private heading = 0;

  travel(ds: number) {
    this.run += ds;
  }

  read(centre: number) {
    this.smoothed =
      this.smoothed === null ? centre : this.smoothed + OFFSET_SMOOTH * (centre - this.smoothed);
    if (this.mark === null) {
      this.mark = this.smoothed;
      this.run = 0;
    } else if (this.run >= MIN_RUN) {
      const crossing = (this.smoothed - this.mark) / this.run;
      this.heading = Math.max(-CROSS_MAX, Math.min(CROSS_MAX, crossing));
      this.mark = this.smoothed;
      this.run = 0;
    }
    return this.heading;
  }
}

export class LaneReader {
  private half: Quadratic | null = null;

  read(lane: Lane): LaneReading {
    const rows = bandEdges(lane);
    const lefts = rows.filter((row) => row.left !== null);
    const rights = rows.filter((row) => row.right !== null);
    this.learn(rows, lefts, rights);

    const midpoint = shapeOf(floorPoints(lane));
    const half = this.half;
    const near = rows.length ? rows[0].d : null;
    if (!half) {
      return { shape: midpoint, opening: null, near };
    }

    const leftCentres = lefts.map((row) => ({ d: row.d, x: row.left! + evaluate(half, row.d) }));
    const rightCentres = rights.map((row) => ({ d: row.d, x: row.right! - evaluate(half, row.d) }));
    const cl = lineFit(leftCentres);
    const cr = lineFit(rightCentres);
    const far = rows.length ? rows.at(-1)!.d : 0;
    const gap = cl && cr ? evaluate(cr, far) - evaluate(cl, far) : null;
    const agree =
      gap !== null && Math.abs(gap) < AGREE_GAP && Math.abs(bend(cr!) - bend(cl!)) < AGREE_BEND;

    let shape = midpoint;
    if (!agree) {
      const fit = lane.anchor === 'right' ? cr : cl;
      const own = lane.anchor === 'right' ? rightCentres : leftCentres;
      shape = fit ? shapeOf([...own, ...rowPoints(lane)]) : null;
    }

    const opening = cl ? this.findOpening(rows, cl, half, gap) : null;
    return { shape, opening, near };
  }

  private findOpening(rows: EdgeRow[], cl: Quadratic, half: Quadratic, gap: number | null) {
    if (Math.abs(bend(cl)) >= STRAIGHT) return null;
    const open = rows.filter((row) => row.left !== null && row.right === null).length;
    if (!((gap !== null && gap > OPEN_GAP) || open >= OPEN_ROWS * rows.length)) return null;
    const out = rows.find(
      (row) =>
        row.left !== null &&
        (row.right === null ||
          row.right - evaluate(cl, row.d) - evaluate(half, row.d) > OUT_OF_PLACE),
    );
    return out ? out.d : null;
  }

  private learn(rows: EdgeRow[], lefts: EdgeRow[], rights: EdgeRow[]) {
    const fl = lineFit(lefts.map((row) => ({ d: row.d, x: row.left! })));
    const fr = lineFit(rights.map((row) => ({ d: row.d, x: row.right! })));
    const both = rows.filter((row) => row.left !== null && row.right !== null);
    if (!fl || !fr || both.length < 10 || Math.abs(bend(fl) + bend(fr)) >= STRAIGHT_PAIR) return;

    const fit = lineFit(both.map((row) => ({ d: row.d, x: (row.right! - row.left!) / 2 })));
    if (!fit) return;
    this.half = this.half
      ? (this.half.map((c, i) => c + LEARN_RATE * (fit[i] - c)) as Quadratic)
      : fit;
  }
}
