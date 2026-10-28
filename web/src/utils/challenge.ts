import type { CartState } from '@/types';

export function isSpeedTest(state: CartState) {
  return state.challenge === 'speed';
}
