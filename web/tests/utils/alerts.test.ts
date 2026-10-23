import { describe, expect, test } from 'vitest';

import type { CartState, SignTrack } from '@/types';
import { worldAlerts } from '@/utils/alerts';

const healthy = (): CartState => ({
  type: 'state',
  version: 1,
  t: 10,
  you: 1,
  drive: {
    mode: 'autonomous',
    estopped: false,
    driver: null,
    lease_lost: false,
    manual_note: null,
    steering: 0.1,
    throttle: 0.5,
    applied_throttle: 0.5,
    speed_est_mps: null,
    stop_text: null,
    stop_source: null,
    fw: {
      state: 'R',
      ms_since_command: 20,
      throttle_pwm: 128,
      steer_pwm: 90,
      angle_raw: 512,
      fault: null,
      age_s: 0.1,
    },
  },
  path: {
    path: 'straight',
    home: 'straight',
    detour_stage: '',
    carrying: false,
    profile: '',
    holding_s: 0,
  },
  lane: {
    state: 'both 5/5',
    offset_px: -4,
    curvature: 0.01,
    anchor: 'auto',
    profile_ready: true,
    provisional: false,
    width_ratio: 0.9,
    rows_found: 5,
    both_rows: 5,
    rows: [],
    frame: [320, 240],
    roi_center_x: 160,
  },
  signs: { camera_open: true, detector: true, status: '', fps: 20, age_s: 0.05, tracks: [] },
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
    reason: 'pickup at A',
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
  lights: { signal: 'off', brake: false, strip: 'off' },
  health: { loop_fps: 30, serial_open: true, drive_camera: true },
  control: { controller: 1, clients: 1 },
});

const track = (overrides: Partial<SignTrack>): SignTrack => ({
  id: 1,
  label: 'traffic_light',
  distance_m: 3.04,
  bearing_deg: 5,
  confirmed: true,
  rejected: false,
  variant: null,
  light: 'green',
  reason: '',
  ...overrides,
});

describe('world alerts', () => {
  test('say nothing while the cart drives normally', () => {
    expect(worldAlerts(healthy())).toEqual([]);
  });
  test('flag an obstacle stop and a silent sonar as bad', () => {
    const s = healthy();
    s.obstacle = { ...s.obstacle, stopped: true, label: 'stop' };
    expect(worldAlerts(s).map((a) => [a.text, a.tone])).toEqual([['Parado por obstáculo', 'bad']]);
    s.obstacle = { ...s.obstacle, stopped: false, label: 'no data' };
    expect(worldAlerts(s).map((a) => [a.text, a.tone])).toEqual([['Sonar sem dados', 'bad']]);
  });
  test('warn while the guard slows for an obstacle', () => {
    const s = healthy();
    s.obstacle = { ...s.obstacle, label: 'slow' };
    expect(worldAlerts(s)).toEqual([{ text: 'Obstáculo · reduzindo', tone: 'warn' }]);
  });
  test('leave confirmed signs to the world view', () => {
    const s = healthy();
    s.signs.tracks = [
      track({}),
      track({ id: 2, label: 'delivery_sign', variant: 'A', distance_m: null }),
      track({ id: 3, label: 'detour' }),
      track({ id: 4, label: 'stop', confirmed: false }),
    ];
    expect(worldAlerts(s)).toEqual([]);
  });
});
