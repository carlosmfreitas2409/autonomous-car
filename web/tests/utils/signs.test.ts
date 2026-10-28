import { describe, expect, test } from 'vitest';

import type { SignTrack } from '@/types';
import { SIGN_SHOW_M, isActionableSign } from '@/utils/signs';

const track = (over: Partial<SignTrack> = {}): SignTrack => ({
  id: 1,
  label: 'stop',
  distance_m: 2.5,
  bearing_deg: 4,
  confirmed: true,
  rejected: false,
  variant: null,
  light: 'unknown',
  reason: '',
  ...over,
});

describe('shown signs', () => {
  test('shows a confirmed sign in range', () => {
    expect(isActionableSign(track())).toBe(true);
    expect(isActionableSign(track({ label: 'detour' }))).toBe(true);
  });
  test('hides what the cart ignores: unconfirmed, rejected, far, unranged', () => {
    expect(isActionableSign(track({ confirmed: false }))).toBe(false);
    expect(isActionableSign(track({ rejected: true }))).toBe(false);
    expect(isActionableSign(track({ distance_m: SIGN_SHOW_M + 0.1 }))).toBe(false);
    expect(isActionableSign(track({ distance_m: null }))).toBe(false);
  });
  test('hides a light until its lamp is read, and a letter until it is read', () => {
    expect(isActionableSign(track({ label: 'traffic_light' }))).toBe(false);
    expect(isActionableSign(track({ label: 'traffic_light', light: 'red' }))).toBe(true);
    expect(isActionableSign(track({ label: 'delivery_sign' }))).toBe(false);
    expect(isActionableSign(track({ label: 'delivery_sign', variant: 'B' }))).toBe(true);
  });
  test('hides labels it does not know', () => {
    expect(isActionableSign(track({ label: 'person' }))).toBe(false);
  });
});
