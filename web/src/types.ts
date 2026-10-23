export type Mode = 'autonomous' | 'manual';
export type PathName = 'normal' | 'straight' | 'detour';
export type Letter = 'A' | 'B' | 'C';
export type FirmwareState = 'R' | 'W' | 'E' | 'F';
export type LightState = 'red' | 'yellow' | 'green' | 'unknown';

export interface Firmware {
  state: FirmwareState;
  ms_since_command: number;
  throttle_pwm: number;
  steer_pwm: number;
  angle_raw: number;
  fault: string | null;
  age_s: number;
}

export type StopSource = 'estop' | 'controller' | 'sign' | 'mission' | 'holding' | 'manual';

export interface Drive {
  mode: Mode;
  estopped: boolean;
  driver: 'pad' | 'dashboard' | null;
  lease_lost: boolean;
  manual_note: string | null;
  steering: number;
  throttle: number;
  applied_throttle: number;
  speed_est_mps: number | null;
  stop_text: string | null;
  stop_source: StopSource | null;
  fw: Firmware | null;
}

export interface Lane {
  state: string;
  offset_px: number | null;
  curvature: number | null;
  anchor: 'auto' | 'left' | 'right';
  profile_ready: boolean;
  provisional: boolean;
  width_ratio: number | null;
  rows_found: number;
  both_rows: number;
  rows: [number, number, boolean][];
  frame: [number, number];
  roi_center_x: number | null;
  band?: [number, number, number][];
  width_profile?: [number, number] | null;
}

export interface SignTrack {
  id: number;
  label: string;
  distance_m: number | null;
  bearing_deg: number | null;
  confirmed: boolean;
  rejected: boolean;
  variant: Letter | null;
  light: LightState;
  reason: string;
}

export interface Signs {
  camera_open: boolean;
  detector: boolean;
  status: string;
  fps: number | null;
  age_s: number | null;
  tracks: SignTrack[];
}

export interface Segment {
  kind: 'main' | 'detour';
  towards?: number;
  exit?: number;
}

export interface Decision {
  exit: number;
  take: 'detour' | 'straight';
}

export type MissionState =
  | 'idle'
  | 'driving'
  | 'slowing'
  | 'docked'
  | 'checking'
  | 'leaving'
  | 'done';

export type DockCheck = 'announcing' | 'lid' | 'weighing' | 'clearing' | 'blocked';

export interface Dock {
  letter: Letter;
  role: 'pickup' | 'delivery' | null;
  since: number;
  released: boolean;
  lid_opened: boolean;
  check_failed: 'empty' | 'still_loaded' | null;
  override: boolean;
}

export interface Cargo {
  grams: number;
  stable: boolean;
  lid_closed: boolean;
  unlocked: boolean;
  fault: boolean;
}

export interface Mission {
  stage: 'idle' | 'pickup' | 'delivery' | 'done';
  pickup: Letter | null;
  delivery: Letter | null;
  target: Letter | null;
  next_exit: number;
  route: string[];
  step: number;
  state: MissionState;
  reason: string;
  scale: number;
  hold: boolean;
  applied: boolean;
  next: (Decision & { target: Letter }) | null;
  decided?: (Decision & { sign: number }) | null;
  segment: Segment | null;
  check: DockCheck | null;
  dock: Dock | null;
  parked_at: Letter | null;
  unlock: boolean;
  manifest_g: number | null;
}

export interface PathStatus {
  path: PathName;
  home: PathName;
  detour_stage: string;
  carrying: boolean;
  profile: string;
  holding_s: number;
}

export interface Policy {
  state: string;
  reason: string;
  scale: number;
  hold: boolean;
  stale: boolean;
  applied: boolean;
}

export interface Obstacle {
  distances_cm: (number | null)[];
  label: 'no data' | 'stale' | 'stop' | 'slow' | 'clear';
  stopped: boolean;
  scale: number;
  stale: boolean;
  slow_cm: number[];
  stop_cm: number;
}

export type TurnSignal = 'off' | 'left' | 'right' | 'hazard';
export type StripEffect =
  | 'off'
  | 'red'
  | 'green'
  | 'blue'
  | 'cyan'
  | 'red_pulse'
  | 'strobe'
  | 'rainbow'
  | 'police'
  | 'blue_breathe'
  | 'cyan_sweep'
  | 'yellow_pulse';

export interface LightSettings {
  turn_signal: TurnSignal;
  drl: boolean;
  low_beam: boolean;
  logo: boolean;
  projector: boolean;
  strip: StripEffect | 'auto';
}

export interface Lights {
  signal: TurnSignal;
  brake: boolean;
  strip: StripEffect;
  requested?: TurnSignal | null;
  settings?: LightSettings | null;
}

export interface SpeedLimits {
  base: number;
  min: number;
  defaults: { base: number; min: number };
}

export type PidGain = 'kp' | 'ki' | 'kd';

export type PidGains = Record<PidGain, number>;

export interface Pid extends PidGains {
  defaults: PidGains;
  max: PidGains;
}

export interface Battery {
  percent: number;
  volts: number;
  age_s: number;
}

export interface CartState {
  type: 'state';
  version: number;
  t: number;
  you: number;
  drive: Drive;
  path: PathStatus;
  lane: Lane | null;
  signs: Signs;
  policy: Policy;
  mission: Mission;
  cargo: Cargo | null;
  obstacle: Obstacle;
  lights: Lights | null;
  health: { loop_fps: number; serial_open: boolean; drive_camera: boolean };
  battery?: Battery | null;
  speed?: SpeedLimits | null;
  pid?: Pid | null;
  control: { controller: number | null; clients: number };
}

export interface TrackLayout {
  exits: number[];
  after: Record<string, number>;
  letters: Record<Letter, number | null>;
}

export interface Hello {
  type: 'hello';
  version: number;
  client: number;
  layout: TrackLayout;
}

export interface Reply {
  type: 'ack' | 'nack';
  id?: number | null;
  cmd?: string | null;
  reason?: string;
}

export type Command =
  | { type: 'estop'; engaged: boolean }
  | { type: 'mode'; mode: Mode }
  | { type: 'drive'; steering: number; throttle: number }
  | { type: 'path'; path: PathName }
  | { type: 'mission'; pickup: Letter | null; delivery: Letter | null }
  | { type: 'dock'; action: 'wait' }
  | { type: 'open_cargo' }
  | ({ type: 'lights' } & Partial<LightSettings>)
  | { type: 'speed'; base?: number; min?: number }
  | ({ type: 'pid' } & Partial<PidGains>)
  | { type: 'claim'; force?: boolean }
  | { type: 'release' }
  | { type: 'ping'; t: number };

export type ServerMessage =
  | CartState
  | Hello
  | Reply
  | { type: 'pong'; t: number }
  | { type: 'events' };
