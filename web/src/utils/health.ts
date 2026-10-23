import { PATH_NAMES, SONAR_NAMES } from '@/constants';
import type { CartState, FirmwareState } from '@/types';

import { formatNumber } from './format';

const FIRMWARE_STATE_NAMES: Record<FirmwareState, string> = {
  R: 'rodando',
  W: 'watchdog',
  E: 'parada',
  F: 'falha',
};

export type Tone = 'ok' | 'warn' | 'bad' | 'none';

export interface Check {
  label: string;
  value: string;
  tone: Tone;
}

export interface HealthInput {
  state: CartState | null;
  isConnected: boolean;
  isStale: boolean;
  ageS: number;
  rttMs: number | null;
}

export interface Health {
  checks: Check[];
  ready: boolean | null;
  blocker: string | null;
}

const FAST_LINK_MS = 60;
const UNKNOWN_ROWS = ['Controlador', 'Sonar', 'Câmera da pista', 'Câmera de placas'];

export function checkHealth({ state, isConnected, isStale, ageS, rttMs }: HealthInput): Health {
  const link = linkCheck(isConnected, isStale, ageS, rttMs);
  if (!state) {
    const unknown = UNKNOWN_ROWS.map((label): Check => ({ label, value: '—', tone: 'none' }));
    return { checks: [link, ...unknown], ready: null, blocker: null };
  }

  const { drive, health, obstacle, signs, mission, path } = state;
  const fw = drive.fw;
  const sonarOk = !obstacle.stale && obstacle.label !== 'no data';

  const checks: Check[] = [
    link,
    controllerCheck(state),
    { label: 'Sonar', value: SONAR_NAMES[obstacle.label], tone: sonarOk ? 'ok' : 'bad' },
    drive.estopped
      ? { label: 'Câmera da pista', value: 'pausada', tone: 'none' }
      : {
          label: 'Câmera da pista',
          value: health.drive_camera ? 'ao vivo' : 'sem imagens',
          tone: health.drive_camera ? 'ok' : 'bad',
        },
    signCameraCheck(state),
  ];

  if (!drive.estopped) return { checks, ready: null, blocker: null };

  const preflight: [boolean, string][] = [
    [rttMs !== null && rttMs < FAST_LINK_MS, 'conexão lenta'],
    [health.serial_open && fw !== null && fw.age_s < 1, 'controlador'],
    [sonarOk, 'sonar'],
    [signs.camera_open && signs.detector, 'câmera de placas'],
    [mission.stage !== 'idle', 'defina uma entrega'],
    [path.path === path.home, `trajeto fora de ${PATH_NAMES[path.home]}`],
  ];
  const blocker = preflight.find(([ok]) => !ok)?.[1] ?? null;
  return { checks, ready: blocker === null, blocker };
}

export function summarizeHealth({ checks, ready, blocker }: Health): { text: string; tone: Tone } {
  if (ready !== null) {
    return ready
      ? { text: 'Pronto para retomar', tone: 'ok' }
      : { text: `Não está pronto · ${blocker}`, tone: 'bad' };
  }
  const bad = checks.filter((check) => check.tone === 'bad').length;
  if (bad > 0) return { text: bad === 1 ? '1 problema' : `${bad} problemas`, tone: 'bad' };
  const warn = checks.filter((check) => check.tone === 'warn').length;
  if (warn > 0) return { text: warn === 1 ? '1 alerta' : `${warn} alertas`, tone: 'warn' };
  return { text: 'Tudo certo', tone: 'ok' };
}

function linkCheck(
  isConnected: boolean,
  isStale: boolean,
  ageS: number,
  rttMs: number | null,
): Check {
  if (!isConnected) return { label: 'Conexão', value: 'sem conexão', tone: 'bad' };
  if (isStale && Number.isFinite(ageS)) {
    return { label: 'Conexão', value: `${formatNumber(ageS, 1)} s de atraso`, tone: 'bad' };
  }
  if (rttMs === null) return { label: 'Conexão', value: '—', tone: 'warn' };
  return {
    label: 'Conexão',
    value: `${Math.round(rttMs)} ms`,
    tone: rttMs < FAST_LINK_MS ? 'ok' : 'warn',
  };
}

function controllerCheck({ drive, health }: CartState): Check {
  const fw = drive.fw;
  if (!health.serial_open) return { label: 'Controlador', value: 'serial fechada', tone: 'bad' };
  if (!fw) return { label: 'Controlador', value: 'sem telemetria', tone: 'bad' };
  if (fw.age_s >= 1) {
    return {
      label: 'Controlador',
      value: `sem resposta há ${formatNumber(fw.age_s, 1)} s`,
      tone: 'bad',
    };
  }
  const fault = fw.fault && fw.fault !== '-' ? ` · falha ${fw.fault}` : '';
  const running = fw.state === 'R' || (fw.state === 'E' && drive.estopped);
  const tone: Tone = fw.state === 'F' ? 'bad' : running ? 'ok' : 'warn';
  return { label: 'Controlador', value: `${FIRMWARE_STATE_NAMES[fw.state]}${fault}`, tone };
}

function signCameraCheck({ signs }: CartState): Check {
  if (!signs.camera_open) {
    return { label: 'Câmera de placas', value: signs.status || 'desligada', tone: 'bad' };
  }
  if (!signs.detector) return { label: 'Câmera de placas', value: 'sem modelo', tone: 'bad' };
  return {
    label: 'Câmera de placas',
    value: `${formatNumber(signs.fps ?? 0)} fps`,
    tone: 'ok',
  };
}
