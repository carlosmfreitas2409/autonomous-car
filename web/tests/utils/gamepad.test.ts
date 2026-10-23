import { describe, expect, test } from 'vitest';

import { fromGamepad, stick } from '@/utils/gamepad';

const pad = (x: number, r2: number, l2: number) => ({
  axes: [x, 0, 0, 0],
  buttons: Array.from({ length: 16 }, (_, i) => ({
    value: i === 7 ? r2 : i === 6 ? l2 : 0,
  })),
});

describe('gamepad', () => {
  test('has a deadzone and reaches full lock', () => {
    expect(stick(0.05)).toBe(0);
    expect(stick(1)).toBe(1);
    expect(stick(-1)).toBe(-1);
  });
  test('maps R2 forward and L2 reverse like the PS5 on the cart', () => {
    expect(fromGamepad(pad(0, 1, 0)).throttle).toBe(1);
    expect(fromGamepad(pad(0, 0, 1)).throttle).toBe(-1);
    expect(fromGamepad(pad(0, 0.5, 0.5)).throttle).toBe(0);
  });
  test('is neutral when nothing is touched', () => {
    expect(fromGamepad(pad(0.02, 0.01, 0))).toEqual({ steering: 0, throttle: 0 });
  });
});
