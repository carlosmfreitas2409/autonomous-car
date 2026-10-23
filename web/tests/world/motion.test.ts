import { describe, expect, test } from 'vitest';

import type { CartState } from '@/types';
import { Trail, carried, groundSpeed } from '@/world/utils/motion';

const drive = (applied: number, estopped = false, estimate: number | null = null) =>
  ({
    drive: { applied_throttle: applied, estopped, speed_est_mps: estimate },
  }) as unknown as CartState;

describe('cart motion for drawing', () => {
  test('is still when stopped or e-stopped, and faster with more drive output', () => {
    expect(groundSpeed(null)).toBe(0);
    expect(groundSpeed(drive(0))).toBe(0);
    expect(groundSpeed(drive(0.5, true))).toBe(0);
    expect(groundSpeed(drive(0.5))).toBeCloseTo(1.2);
    expect(groundSpeed(drive(0.63))).toBeGreaterThan(groundSpeed(drive(0.5)));
    expect(groundSpeed(drive(-0.5))).toBeLessThan(0);
  });
  test("prefers the cart's own speed estimate", () => {
    expect(groundSpeed(drive(0.5, false, 1.0))).toBeCloseTo(2);
  });
  test('carries what it passes behind it, and swings it the other way as it turns', () => {
    const p = carried(0.8, 1, 1.5, 0);
    expect(p.x).toBeCloseTo(0.8);
    expect(p.z).toBeCloseTo(-0.5);
    expect(carried(0, 3, 0, 0.2).x).toBeLessThan(0);
  });
});

describe('the road behind the cart', () => {
  const drive = (ds: number, turn: number, frames: number, keep = 1.6) => {
    const trail = new Trail();
    for (let i = 0; i < frames; i++) trail.advance(ds, turn, keep);
    return trail;
  };

  test('runs straight back when the cart has driven straight', () => {
    const trail = drive(0.05, 0, 80);
    for (const s of [-0.3, -0.8, -1.4]) {
      const p = trail.at(s)!;
      expect(p.x).toBeCloseTo(0, 6);
      expect(p.z).toBeCloseTo(s, 6);
      expect(p.angle).toBeCloseTo(0, 6);
    }
  });
  test('keeps the bend the cart has come round, off to the side it turned from', () => {
    const trail = drive(0.05, 0.05 * 0.42, 80);
    const p = trail.at(-1.6)!;
    const turned = 1.6 * 0.42;
    expect(p.x).toBeCloseTo((1 - Math.cos(turned)) / 0.42, 1);
    expect(p.z).toBeCloseTo(-Math.sin(turned) / 0.42, 1);
    expect(p.angle).toBeCloseTo(-turned, 1);
    const mirrored = drive(0.05, -0.05 * 0.42, 80).at(-1.6)!;
    expect(mirrored.x).toBeCloseTo(-p.x, 6);
    expect(mirrored.angle).toBeCloseTo(-p.angle, 6);
  });
  test('says nothing about road the cart has not driven yet', () => {
    expect(drive(0.05, 0, 4).at(-1.5)).toBeNull();
    expect(new Trail().at(-0.5)).toBeNull();
  });
  test('does not pile up while the cart stands still', () => {
    const trail = drive(0.05, 0, 40);
    const stopped = drive(0.05, 0, 40);
    for (let i = 0; i < 5000; i++) stopped.advance(0, 0, 1.6);
    expect(stopped.at(-1.0)!.z).toBeCloseTo(trail.at(-1.0)!.z, 6);
  });
});
