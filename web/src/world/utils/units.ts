import type { SignTrack } from '@/types';

export const LANE_WIDTH_M = 1.0;
export const LANE_REAL_M = 0.5;

export const LANE_SEEN_M = 2.0;
export const LANE_FADE_M = 5;
export const LANE_BEHIND_M = 1.6;

export const SIGN_SETBACK_M = 0.35;

export const SONAR_BEARINGS_DEG = [-28, 0, 28];

export const LOCK_RADIUS = 0.6 / LANE_REAL_M;

export function signPlace(track: SignTrack) {
  if (track.distance_m === null) return null;
  const bearing = ((track.bearing_deg ?? 0) * Math.PI) / 180;
  const reach = track.distance_m / LANE_REAL_M;
  return { x: reach * Math.sin(bearing), z: reach * Math.cos(bearing) };
}
