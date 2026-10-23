import { describe, expect, test } from 'vitest';

import type { Lane } from '@/types';
import {
  ART_SCALE,
  CAR_CENTRE_Z,
  chaseCamera,
  project,
  projectArtPoint,
} from '@/world/utils/camera';
import { laneCentre } from '@/world/utils/lane-fit';
import { BEND_IN, across, mergePath, pathReach, roadCurve } from '@/world/utils/road';
import { sonarZone } from '@/world/utils/sonar';
import { LANE_BEHIND_M, LANE_FADE_M, LANE_REAL_M, signPlace } from '@/world/utils/units';

const lane = (offset: number | null, curvature = 0): Lane => ({
  state: 'both 3/5',
  offset_px: offset,
  curvature,
  anchor: 'auto',
  profile_ready: true,
  provisional: false,
  width_ratio: null,
  rows_found: 3,
  both_rows: 3,
  rows: [],
  frame: [320, 240],
  roi_center_x: 160,
});

describe('world view', () => {
  test('puts the lane centre right of the cart for a positive offset', () => {
    expect(laneCentre(lane(40))).toBeGreaterThan(0);
    expect(laneCentre(lane(-40))).toBeLessThan(0);
    expect(laneCentre(lane(null))).toBe(0);
  });
  test('places a sign at its range and bearing', () => {
    const p = signPlace({
      id: 1,
      label: 'stop',
      distance_m: 2,
      bearing_deg: 30,
      confirmed: true,
      rejected: false,
      variant: null,
      light: 'unknown',
      reason: '',
    })!;
    expect(p.x * LANE_REAL_M).toBeCloseTo(1);
    expect(p.z * LANE_REAL_M).toBeCloseTo(Math.sqrt(3));
  });
  test('projects nearer ground lower on screen', () => {
    const cam = chaseCamera(800, 600);
    expect(project(cam, 0, 1)!.y).toBeGreaterThan(project(cam, 0, 5)!.y);
  });
  test("draws the ground where the cart's art expects it", () => {
    for (const [w, h] of [
      [990, 490],
      [1600, 646],
      [800, 600],
    ]) {
      const cam = chaseCamera(w, h);
      for (const [x, z] of [
        [0.4, 0.485],
        [-0.4, -0.496],
        [0, 0],
        [0.9, 3],
        [-1.2, 8],
      ]) {
        const art = projectArtPoint(x, 0, z)!;
        const p = project(cam, x * ART_SCALE, z * ART_SCALE + CAR_CENTRE_Z)!;
        expect(p.x).toBeCloseTo(cam.car[0] + art[0] * cam.artScale, 6);
        expect(p.y).toBeCloseTo(cam.car[1] + art[1] * cam.artScale, 6);
      }
    }
  });
  test('keeps the road the cart stands on square with the cart', () => {
    for (const curvature of [-0.9, -0.4, 0, 0.4, 0.9]) {
      const curve = roadCurve({ centre: 0.1, heading: 0.05, curvature }, LANE_FADE_M);
      const beside = Math.atan(0.05);
      for (let s = -LANE_BEHIND_M; s <= 0; s += 0.1) {
        expect(curve(s).angle).toBeCloseTo(beside, 10);
      }
    }
  });
  test("is bending by the time the road clears the cart's art", () => {
    const k = 0.42;
    const curve = roadCurve({ centre: 0, heading: 0, curvature: k }, LANE_FADE_M);
    const rateAt = (s: number) => (curve(s + 0.05).angle - curve(s - 0.05).angle) / 0.1;
    expect(rateAt(0.45) / k).toBeGreaterThan(0.8);
    expect(rateAt(0.25) / k).toBeGreaterThan(0.3);
  });
  test('runs straight back behind the cart, where it has been', () => {
    const curve = roadCurve({ centre: 0, heading: 0.1, curvature: 0.8 }, LANE_FADE_M);
    expect(curve(-1).angle).toBeCloseTo(curve(0).angle, 10);
    expect(curve(-1).z).toBeCloseTo(-Math.cos(Math.atan(0.1)), 6);
  });
  test('turns further the further out the road goes', () => {
    const curve = roadCurve({ centre: 0, heading: 0, curvature: 0.42 }, LANE_FADE_M);
    const turnedBy = (s: number) => curve(s).angle;
    expect(turnedBy(BEND_IN)).toBeGreaterThan(turnedBy(0));
    expect(turnedBy(1.2)).toBeGreaterThan(turnedBy(BEND_IN));
    expect(turnedBy(2.6)).toBeGreaterThan(0.6);
  });
  test('keeps its length whatever the bend, instead of pumping shorter', () => {
    for (const curvature of [0, 0.5, 0.9]) {
      const curve = roadCurve({ centre: 0, heading: 0, curvature }, LANE_FADE_M);
      let along = 0;
      for (let s = 0; s + 0.1 <= LANE_FADE_M; s += 0.1) {
        const a = curve(s);
        const b = curve(s + 0.1);
        along += Math.hypot(b.x - a.x, b.z - a.z);
      }
      expect(along).toBeCloseTo(LANE_FADE_M, 1);
    }
  });
  test('stops bending rather than curling into a hairpin', () => {
    const curve = roadCurve({ centre: 0, heading: 0, curvature: 5 }, LANE_FADE_M);
    expect(curve(LANE_FADE_M).angle).toBeLessThan(1.4);
  });
  test('draws the path onto the lane without a crease in it', () => {
    const cases = [
      { centre: 0.12, heading: 0.03, curvature: -0.45 },
      { centre: 0.19, heading: 0.05, curvature: -0.42 },
      { centre: -0.15, heading: -0.03, curvature: 0.42 },
      { centre: 0.05, heading: 0, curvature: -0.2 },
      { centre: 0, heading: 0, curvature: 0 },
    ];
    for (const road of cases) {
      const curve = roadCurve(road, LANE_FADE_M);
      const pts = mergePath(curve, -road.centre, 0.25, 2.6, 2.0, 48);
      expect(Math.abs(pts[0].x)).toBeLessThan(0.05);
      const dir = pts.map((_, k) => {
        const b = pts[Math.max(0, k - 1)];
        const a = pts[Math.min(pts.length - 1, k + 1)];
        return Math.atan2(a.x - b.x, a.z - b.z);
      });
      for (let k = 1; k < dir.length; k++) {
        expect(Math.abs(dir[k] - dir[k - 1])).toBeLessThan(0.045);
      }
    }
  });
  test("lays the path's offset across the road, not across the picture", () => {
    const p = { x: 1, z: 2, angle: Math.PI / 2 };
    const right = across(p, 0.5);
    expect(right.x).toBeCloseTo(1, 10);
    expect(right.z).toBeCloseTo(1.5, 10);
    expect(across(p, 0)).toEqual({ x: 1, z: 2 });
  });
  test('draws the path only as far as the cart is committed', () => {
    expect(pathReach(1.2, LANE_FADE_M)).toBeCloseTo(2.64, 2);
    expect(pathReach(0.6, LANE_FADE_M)).toBeCloseTo(1.32, 2);
    expect(pathReach(1.2, 1.5)).toBe(1.5);
    expect(pathReach(0.9, 2.6)!).toBeLessThan(pathReach(1.2, 2.6)!);
    expect(pathReach(0.15, 2.6)).toBeNull();
    expect(pathReach(0, 2.6)).toBeNull();
  });
  test("uses the guard's thresholds", () => {
    expect(sonarZone(50, 100, 60)).toBe('stop');
    expect(sonarZone(80, 100, 60)).toBe('slow');
    expect(sonarZone(140, 100, 60)).toBe('clear');
    expect(sonarZone(null, 100, 60)).toBe('clear');
  });
});
