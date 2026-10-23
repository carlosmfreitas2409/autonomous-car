import { describe, expect, test } from 'vitest';

import type { CartState, Lane, SignTrack } from '@/types';
import { type Branch, ForkWatch, branchAt } from '@/world/utils/fork-watch';
import { laneShape } from '@/world/utils/lane-fit';
import { LaneReader, type LaneReading } from '@/world/utils/lane-reader';

import fixtures from '../fixtures/forks.json';

interface Frame {
  i: number;
  path: string;
  offset: number | null;
  slope: number | null;
  rows: number[][];
  band: number[][];
  profile: number[] | null;
}

const toLane = (f: Frame): Lane => ({
  state: 'anchor 5/5',
  offset_px: f.offset,
  curvature: f.slope,
  anchor: f.path === 'detour' ? 'right' : 'left',
  profile_ready: true,
  provisional: false,
  width_ratio: null,
  rows_found: f.rows.length,
  both_rows: 0,
  rows: f.rows.map(([y, x]) => [y, x, false]),
  frame: [960, 540],
  roi_center_x: 485,
  band: f.band as [number, number, number][],
  width_profile: f.profile as [number, number] | null,
});

function readRun(frames: Frame[]): Map<number, { lane: Lane; reading: LaneReading }> {
  const reader = new LaneReader();
  return new Map(
    frames.map((f) => {
      const lane = toLane(f);
      return [f.i, { lane, reading: reader.read(lane) }];
    }),
  );
}

const day10 = readRun(fixtures.day10);
const day8 = readRun(fixtures.day8);
const span = (run: typeof day10, lo: number, hi: number) =>
  [...run].filter(([i]) => i >= lo && i <= hi).map(([, v]) => v);

describe('lane reading at a fork', () => {
  test('reads a straight like the midpoint does', () => {
    for (const { lane, reading } of span(day10, 320, 372)) {
      expect(reading.shape!.curvature).toBeCloseTo(laneShape(lane)!.curvature, 5);
      expect(reading.opening).toBeNull();
    }
  });
  test('keeps the main road straight past a detour the midpoint took for a right turn', () => {
    const fork = span(day10, 384, 424);
    expect(Math.max(...fork.map(({ lane }) => laneShape(lane)?.curvature ?? 0))).toBeGreaterThan(
      0.4,
    );
    for (const { reading } of fork) {
      expect(Math.abs(reading.shape!.curvature)).toBeLessThan(0.12);
    }
  });
  test('finds the opening and follows it in towards the cart', () => {
    const openings = span(day10, 384, 424).map(({ reading }) => reading.opening);
    expect(openings.filter((d) => d !== null).length).toBeGreaterThanOrEqual(openings.length - 2);
    const first = openings.find((d) => d !== null)!;
    const last = openings.filter((d) => d !== null).at(-1)!;
    expect(first).toBeGreaterThan(last + 0.2);
  });
  test('sees a bend, not a fork, where the right line leaves the frame with the left one bending', () => {
    const bend = span(day10, 492, 560);
    for (const { reading } of bend) expect(reading.opening).toBeNull();
    expect(
      bend.filter(({ reading }) => (reading.shape?.curvature ?? 0) > 0.3).length,
    ).toBeGreaterThan(bend.length / 2);
  });
  test('turns into the detour on the right line', () => {
    const entry = span(day8, 740, 752);
    for (const { reading } of entry) {
      expect(reading.shape!.curvature).toBeGreaterThan(0.3);
      expect(reading.opening).not.toBeNull();
    }
  });
  test('does not bend the lane on the noise of a short view before the detour', () => {
    for (const { reading } of span(day8, 704, 732))
      expect(Math.abs(reading.shape?.curvature ?? 0)).toBeLessThan(0.15);
  });
});

const detourSign = (id: number, distance: number): SignTrack => ({
  id,
  label: 'detour',
  distance_m: distance,
  bearing_deg: 8,
  confirmed: true,
  rejected: false,
  variant: null,
  light: 'unknown',
  reason: '',
});

const cartState = (
  over: {
    lane?: Lane | null;
    tracks?: SignTrack[];
    path?: 'straight' | 'detour';
    stage?: string;
    next?: Branch | null;
    decided?: { sign: number; take: Branch } | null;
  } = {},
): CartState => ({
  type: 'state',
  version: 1,
  t: 0,
  you: 1,
  drive: {
    mode: 'autonomous',
    estopped: false,
    driver: null,
    lease_lost: false,
    manual_note: null,
    steering: 0,
    throttle: 0.4,
    applied_throttle: 0.4,
    speed_est_mps: null,
    stop_text: null,
    stop_source: null,
    fw: null,
  },
  path: {
    path: over.path ?? 'straight',
    home: 'straight',
    detour_stage: over.stage ?? '',
    carrying: false,
    profile: '',
    holding_s: 0,
  },
  lane: over.lane === undefined ? toLane(fixtures.day10[0]) : over.lane,
  signs: {
    camera_open: true,
    detector: true,
    status: '',
    fps: 20,
    age_s: 0.05,
    tracks: over.tracks ?? [],
  },
  policy: { state: 'clear', reason: '', scale: 1, hold: false, stale: false, applied: true },
  mission: {
    stage: 'pickup',
    pickup: 'A',
    delivery: 'C',
    target: 'A',
    next_exit: 1,
    route: [],
    step: 0,
    state: 'driving',
    reason: '',
    scale: 1,
    hold: false,
    applied: true,
    check: null,
    dock: null,
    parked_at: null,
    unlock: false,
    manifest_g: null,
    segment: null,
    next: over.next === null ? null : { exit: 1, take: over.next ?? 'detour', target: 'A' },
    decided: over.decided ? { sign: over.decided.sign, exit: 1, take: over.decided.take } : null,
  },
  cargo: null,
  obstacle: {
    distances_cm: [200, 200, 200],
    label: 'clear',
    stopped: false,
    scale: 1,
    stale: false,
    slow_cm: [100, 150, 100],
    stop_cm: 60,
  },
  lights: null,
  health: { loop_fps: 20, serial_open: true, drive_camera: true },
  control: { controller: 1, clients: 1 },
});

const quiet: LaneReading = { shape: null, opening: null, near: 0.17 };
const open = (d: number): LaneReading => ({ ...quiet, opening: d });

describe('the fork the world view draws', () => {
  test('takes the branch the mission planned until it decides that sign, then the path', () => {
    expect(branchAt(cartState({ next: 'detour' }), 5)).toBe('detour');
    expect(
      branchAt(
        cartState({ next: 'straight', decided: { sign: 5, take: 'detour' }, path: 'detour' }),
        5,
      ),
    ).toBe('detour');
    expect(branchAt(cartState({ next: 'detour', decided: { sign: 5, take: 'straight' } }), 5)).toBe(
      'straight',
    );
    expect(branchAt(cartState({ next: null }), null)).toBe('straight');
  });
  test('draws nothing where the camera alone sees the right line go', () => {
    const watch = new ForkWatch();
    for (let k = 0; k < 40; k++)
      expect(watch.update(cartState(), open(0.3), 0.05, k * 0.05)).toBeNull();
  });
  test('puts the fork at a confirmed detour sign, then where the camera finds it, then behind the cart', () => {
    const watch = new ForkWatch();
    let t = 0;
    let view = null;
    for (let k = 0; k < 20; k++, t += 0.05)
      view = watch.update(cartState({ tracks: [detourSign(5, 3.0)] }), quiet, 0.05, t);
    expect(view!.at).toBeGreaterThan(2.5);
    expect(view!.take).toBe('detour');
    expect(view!.alpha).toBeGreaterThan(0.9);
    for (let k = 0; k < 20; k++, t += 0.05)
      view = watch.update(cartState(), open(0.45), 0.05, t, true, 0.06);
    expect(view!.at).toBeLessThan(0.6);
    for (let k = 0; k < 20; k++, t += 0.05)
      view = watch.update(cartState(), open(0.17), 0.05, t, true, 0.06);
    expect(view!.at).toBeLessThan(-0.5);
    for (let k = 0; k < 60; k++, t += 0.05)
      view = watch.update(cartState({ tracks: [detourSign(5, 3.0)] }), quiet, 0.05, t);
    expect(view).toBeNull();
  });
  test('waits for the camera when only an armed Detour says a fork is coming', () => {
    const watch = new ForkWatch();
    const waiting = cartState({ path: 'detour', stage: 'waiting for the entrance' });
    expect(watch.update(waiting, quiet, 0.05, 0)).toBeNull();
    let view = null;
    for (let k = 1; k < 12; k++) view = watch.update(waiting, open(0.3), 0.05, k * 0.05);
    expect(view!.take).toBe('detour');
    expect(view!.at).toBeGreaterThan(0.2);
  });
  test('stays where it is while the cart stands still', () => {
    const watch = new ForkWatch();
    let view = null;
    for (let t = 0; t < 1; t += 0.05)
      view = watch.update(cartState({ tracks: [detourSign(5, 3.0)] }), quiet, 0.05, t);
    for (let t = 1; t < 4; t += 0.05) view = watch.update(cartState(), quiet, 0.05, t, true, 0);
    expect(view!.at).toBeGreaterThan(5);
  });
  test('gives a sign up when the camera never finds its fork', () => {
    const watch = new ForkWatch();
    watch.update(cartState({ tracks: [detourSign(9, 3.5)] }), quiet, 0.05, 0);
    let view = null;
    for (let t = 0.05; t < 12; t += 0.05) view = watch.update(cartState(), quiet, 0.05, t);
    expect(view).toBeNull();
  });
});
