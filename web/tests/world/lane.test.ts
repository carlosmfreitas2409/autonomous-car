import { describe, expect, test } from 'vitest';

import type { Lane } from '@/types';
import { laneCentre, laneShape } from '@/world/utils/lane-fit';
import { LaneHeading } from '@/world/utils/lane-reader';
import { lanePoint } from '@/world/utils/road';

const lane = (offset: number | null, slope: number): Lane => ({
  state: 'both 5/5',
  offset_px: offset,
  curvature: slope,
  anchor: 'auto',
  profile_ready: true,
  provisional: false,
  width_ratio: null,
  rows_found: 5,
  both_rows: 5,
  rows: [],
  frame: [480, 270],
  roi_center_x: 240,
});

const frame = (
  rows: [number, number][],
  band: [number, number, number][],
  profile: [number, number],
): Lane => ({
  state: 'anchor-left 5/5',
  offset_px: 0,
  curvature: 0,
  anchor: 'left',
  profile_ready: true,
  provisional: false,
  width_ratio: null,
  rows_found: rows.length,
  both_rows: 0,
  rows: rows.map(([y, x]) => [y, x, false]),
  frame: [960, 540],
  roi_center_x: 480,
  band,
  width_profile: profile,
});
const STRAIGHT = frame(
  [
    [471, 484],
    [435, 479],
    [398, 478],
    [362, 478],
  ],
  [
    [357, 92, 864],
    [345, 101, 854],
    [333, 110, 844],
    [321, 119, 834],
    [309, 129, 824],
    [297, 139, 813],
    [285, 149, 802],
    [273, 159, 791],
    [261, 169, 780],
    [249, 180, 768],
    [237, 190, 757],
    [225, 201, 745],
    [213, 213, 733],
    [201, 225, 720],
    [189, 237, 708],
  ],
  [1.4517, 254.4949],
);
const RIGHT_BEND = frame(
  [
    [508, 445],
    [471, 447],
    [435, 443],
    [398, 440],
    [362, 439],
  ],
  [
    [357, 70, 805],
    [345, 78, 795],
    [333, 86, 786],
    [321, 96, 777],
    [309, 106, 769],
    [297, 116, 760],
    [285, 127, 753],
    [273, 138, 746],
    [261, 150, 739],
    [249, 162, 733],
    [237, 175, 728],
    [225, 189, 724],
    [213, 204, 722],
    [201, 220, 721],
    [189, 236, 723],
  ],
  [1.3542, 255.5697],
);
const LEFT_BEND = frame(
  [
    [508, 437],
    [471, 435],
    [435, 424],
    [398, 413],
    [362, 399],
  ],
  [
    [357, 33, 749],
    [345, 35, 736],
    [333, 36, 723],
    [321, 37, 709],
    [309, 36, 695],
    [297, 34, 681],
    [285, 30, 665],
    [273, 22, 648],
    [261, 7, 631],
  ],
  [1.1653, 309.0492],
);

describe('lane drawing', () => {
  test("reads the cart's offset off the tracker", () => {
    expect(laneCentre(lane(20, 0.3))).toBeGreaterThan(0);
    expect(laneCentre(lane(-20, 0.3))).toBeLessThan(0);
  });
  test('reads a straight as straight', () => {
    const shape = laneShape(STRAIGHT)!;
    expect(Math.abs(shape.curvature)).toBeLessThan(0.05);
  });
  test('bends right and left where the track does', () => {
    expect(laneShape(RIGHT_BEND)!.curvature).toBeGreaterThan(0.2);
    expect(laneShape(LEFT_BEND)!.curvature).toBeLessThan(-0.2);
  });
  test('says nothing without the band or the width profile', () => {
    expect(laneShape({ ...STRAIGHT, band: [], rows: [] })).toBeNull();
    expect(laneShape({ ...STRAIGHT, width_profile: null })).toBeNull();
  });
  test('lays a bend out as an arc', () => {
    const straight = lanePoint(0.1, 0, 0, 2);
    expect(straight.x).toBeCloseTo(0.1);
    expect(straight.z).toBeCloseTo(2);
    const quarter = lanePoint(0, 0, 0.5, Math.PI);
    expect(quarter.x).toBeCloseTo(2);
    expect(quarter.z).toBeCloseTo(2);
    const half = lanePoint(0, 0, 0.5, 2 * Math.PI);
    expect(half.x).toBeCloseTo(4);
    expect(half.z).toBeCloseTo(0, 5);
  });
});

describe('heading from the offset', () => {
  const drive = (rate: number, steps = 300, ds = 0.06) => {
    const heading = new LaneHeading();
    let offset = 0;
    let last = 0;
    for (let i = 0; i < steps; i++) {
      heading.travel(ds);
      offset += rate * ds;
      last = heading.read(offset);
    }
    return last;
  };

  test('is nothing while the cart holds its line', () => {
    expect(drive(0)).toBeCloseTo(0, 6);
  });
  test('is the rate the cart is crossing the lane', () => {
    expect(drive(0.1)).toBeCloseTo(0.1, 2);
    expect(drive(-0.15)).toBeCloseTo(-0.15, 2);
  });
  test('never reports a cart crossing faster than one can', () => {
    const heading = new LaneHeading();
    let worst = 0;
    for (let i = 0; i < 200; i++) {
      heading.travel(0.06);
      worst = Math.max(worst, Math.abs(heading.read(i % 2 ? 0.4 : -0.4)));
    }
    expect(worst).toBeLessThanOrEqual(0.3);
  });
  test('holds what it knew while the cart is stopped', () => {
    const heading = new LaneHeading();
    for (let i = 0; i < 200; i++) {
      heading.travel(0.06);
      heading.read(i * 0.06 * 0.1);
    }
    const moving = heading.read(1.2);
    for (let i = 0; i < 50; i++) expect(heading.read(1.2)).toBe(moving);
  });
});
