import type { SignTrack } from '@/types';

export const SIGN_SHOW_M = 4;

const KNOWN_LABELS = new Set(['stop', 'detour', 'traffic_light', 'delivery_sign']);

export function isActionableSign(track: SignTrack) {
  if (!KNOWN_LABELS.has(track.label) || !track.confirmed || track.rejected) return false;
  if (track.distance_m === null || track.distance_m > SIGN_SHOW_M) return false;
  if (track.label === 'traffic_light' && track.light === 'unknown') return false;
  if (track.label === 'delivery_sign' && track.variant === null) return false;
  return true;
}
