import type { Obstacle, PathName, StripEffect } from '@/types';

export const CONTRACT_VERSION = 2;

export const STALE_MS = 1000;

export const PATH_NAMES: Record<PathName, string> = {
  normal: 'Normal',
  straight: 'Reto',
  detour: 'Desvio',
};

export const SONAR_NAMES: Record<Obstacle['label'], string> = {
  clear: 'livre',
  slow: 'reduzindo',
  stop: 'parado',
  stale: 'atrasado',
  'no data': 'sem dados',
};

export const CONTROL_SOCKET_PATH = '/ws/control';
export const VIDEO_SOCKET_PATH = '/ws/video';

export const STRIP_COLOURS: Record<StripEffect, string | null> = {
  off: null,
  red: '#ff3b30',
  red_pulse: '#ff3b30',
  green: '#34c759',
  blue: '#2f6dff',
  cyan: '#22c8e5',
  police: '#2f6dff',
  strobe: '#ffffff',
  rainbow: '#b45cff',
  blue_breathe: '#2f6dff',
  cyan_sweep: '#22c8e5',
  yellow_pulse: '#ffcc00',
};
