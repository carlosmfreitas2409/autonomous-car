import { describe, expect, test } from 'vitest';

import type { Mission, TrackLayout } from '@/types';
import {
  legNodes,
  letterPositions,
  roads,
  routeLegs,
} from '@/components/delivery-panel/utils/track-geometry';

const venue: TrackLayout = {
  exits: [1, 2],
  after: { '1': 2, '2': 1 },
  letters: { A: 1, B: null, C: null },
};
const final: TrackLayout = { ...venue, letters: { A: 1, B: 2, C: null } };

describe('track map', () => {
  test('has a road for every piece of the track', () => {
    const keys = Object.keys(roads(venue));
    expect(keys.sort()).toEqual(['detour-1', 'detour-2', 'main-1', 'main-2']);
  });
  test('moves B into detour 2 when the layout says so', () => {
    const atVenue = letterPositions(venue);
    const onFinal = letterPositions(final);
    expect(onFinal.B.x).toBeLessThan(atVenue.B.x);
    expect(onFinal.B.x).toBeGreaterThan(onFinal.A.x);
    expect(onFinal.A).toEqual(atVenue.A);
  });
  test('routes A to B at the venue: into detour 1, back round, straight past both exits', () => {
    expect(legNodes(venue, null, 'A')).toEqual(['start', 'exit-1', 'mid-1']);
    expect(legNodes(venue, 'A', 'B')).toEqual([
      'mid-1',
      'merge-1',
      'exit-1',
      'exit-2',
      'right',
      'letter-B',
    ]);
  });
  test('routes B to C on the final day back round, since detour 2 skips C', () => {
    expect(legNodes(final, null, 'B')).toEqual(['start', 'exit-1', 'exit-2', 'mid-2']);
    expect(legNodes(final, 'B', 'C')).toEqual([
      'mid-2',
      'merge-2',
      'merge-1',
      'exit-1',
      'exit-2',
      'right',
      'letter-C',
    ]);
  });
  test('shows the leg being driven and the next, never one already driven', () => {
    const mission = (stage: Mission['stage'], step: number): Mission => ({
      stage,
      pickup: 'A',
      delivery: 'B',
      target: stage === 'pickup' ? 'A' : 'B',
      next_exit: 1,
      step,
      route: ['exit 1: detour', 'A: pickup', 'exit 1: straight', 'exit 2: straight', 'B: delivery'],
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
      next: null,
      segment: { kind: 'main', towards: 1 },
    });
    const before = routeLegs(venue, mission('pickup', 0));
    expect(before.current).toMatch(/^M 440 135 /);
    expect(before.next).toMatch(/^M 890 560 /);
    const delivering = routeLegs(venue, mission('delivery', 3));
    expect(delivering.current).toMatch(/^M 700 135 /);
    expect(delivering.next).toBeNull();
    expect(routeLegs(venue, mission('done', 5))).toEqual({ current: null, next: null });
  });
});
