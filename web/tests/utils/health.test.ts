import { describe, expect, test } from 'vitest';

import type { CartState } from '@/types';
import { checkHealth } from '@/utils/health';

const state = (
  overrides: { estopped?: boolean; stage?: 'idle' | 'pickup'; path?: 'straight' | 'detour' } = {},
): CartState => ({
  type: 'state',
  version: 1,
  t: 10,
  you: 1,
  drive: {
    mode: 'autonomous',
    estopped: overrides.estopped ?? false,
    driver: null,
    lease_lost: false,
    manual_note: null,
    steering: 0,
    throttle: 0,
    applied_throttle: 0,
    speed_est_mps: null,
    stop_text: null,
    stop_source: null,
    fw: {
      state: 'R',
      ms_since_command: 20,
      throttle_pwm: 0,
      steer_pwm: 90,
      angle_raw: 512,
      fault: null,
      age_s: 0.1,
    },
  },
  path: {
    path: overrides.path ?? 'straight',
    home: 'straight',
    detour_stage: '',
    carrying: false,
    profile: '',
    holding_s: 0,
  },
  lane: null,
  signs: { camera_open: true, detector: true, status: '', fps: 20, age_s: 0.05, tracks: [] },
  policy: { state: 'clear', reason: '', scale: 1, hold: false, stale: false, applied: true },
  mission: {
    stage: overrides.stage ?? 'pickup',
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
    next: null,
    segment: null,
  },
  cargo: null,
  obstacle: {
    distances_cm: [180, 220, 190],
    label: 'clear',
    stopped: false,
    scale: 1,
    stale: false,
    slow_cm: [100, 150, 100],
    stop_cm: 60,
  },
  lights: null,
  health: { loop_fps: 30, serial_open: true, drive_camera: true },
  control: { controller: 1, clients: 1 },
});

const input = (s: CartState | null, more: Partial<Parameters<typeof checkHealth>[0]> = {}) => ({
  state: s,
  isConnected: true,
  isStale: false,
  ageS: 0.05,
  rttMs: 4,
  ...more,
});

const tone = (r: ReturnType<typeof checkHealth>, label: string) =>
  r.checks.find((c) => c.label === label)?.tone;

describe('system status', () => {
  test('is ready to resume when every preflight check passes', () => {
    expect(checkHealth(input(state({ estopped: true }))).ready).toBe(true);
  });
  test('is not ready without letters, and says why', () => {
    const r = checkHealth(input(state({ estopped: true, stage: 'idle' })));
    expect(r.ready).toBe(false);
    expect(r.blocker).toBe('defina uma entrega');
  });
  test('is not ready off the home path', () => {
    const r = checkHealth(input(state({ estopped: true, path: 'detour' })));
    expect(r.ready).toBe(false);
    expect(r.blocker).toBe('trajeto fora de Reto');
  });
  test('shows five rows, the battery has its own gauge', () => {
    expect(checkHealth(input(state())).checks.map((c) => c.label)).toEqual([
      'Conexão',
      'Controlador',
      'Sonar',
      'Câmera da pista',
      'Câmera de placas',
    ]);
  });
  test('only judges the pre-resume conditions while e-stopped', () => {
    const r = checkHealth(input(state({ path: 'detour' })));
    expect(r.ready).toBeNull();
    expect(r.blocker).toBeNull();
  });
  test('flags a lost link and has no verdict without a state', () => {
    const r = checkHealth(input(null, { isConnected: false }));
    expect(tone(r, 'Conexão')).toBe('bad');
    expect(r.ready).toBeNull();
  });
});
