import { SONAR_NAMES } from '@/constants';
import type { CartState } from '@/types';

import { isSpeedTest } from './challenge';
import { missionLine, policyText } from './loop-text';

export interface Alert {
  text: string;
  tone: 'warn' | 'bad';
  detail?: string;
}

const MAX_ALERTS = 6;

export function worldAlerts(state: CartState): Alert[] {
  const { lane } = state;
  const alerts = isSpeedTest(state) ? [] : deliveryAlerts(state);

  if (lane && (lane.state.startsWith('LOST') || lane.offset_px === null)) {
    alerts.push({ text: 'Faixa perdida', tone: 'bad', detail: lane.state });
  } else if (lane && !lane.profile_ready && state.path.path !== 'normal') {
    alerts.push({ text: 'Calibrando a largura da faixa', tone: 'warn', detail: lane.state });
  }

  return alerts.slice(0, MAX_ALERTS);
}

function deliveryAlerts(state: CartState): Alert[] {
  const { policy, obstacle, signs, mission } = state;
  const alerts: Alert[] = [];

  if (policy.state !== 'clear') {
    alerts.push({
      text: policyText(policy),
      tone: policy.hold ? 'bad' : 'warn',
      detail: policy.applied ? '' : 'não aplicado',
    });
  }
  const isAtDock = ['docked', 'checking', 'done'].includes(mission.state);
  if ((mission.hold && !isAtDock) || mission.state === 'slowing') {
    alerts.push({ text: missionLine(mission), tone: mission.hold ? 'bad' : 'warn' });
  }

  if (obstacle.stopped) {
    alerts.push({
      text: 'Parado por obstáculo',
      tone: 'bad',
      detail: state.drive.mode === 'manual' ? 'não aplicado · manual' : '',
    });
  } else if (obstacle.label === 'slow') {
    alerts.push({ text: 'Obstáculo · reduzindo', tone: 'warn' });
  }
  if (obstacle.stale || obstacle.label === 'no data') {
    alerts.push({ text: `Sonar ${SONAR_NAMES[obstacle.label]}`, tone: 'bad' });
  }

  if (!signs.camera_open) {
    alerts.push({ text: 'Câmera de placas desligada', tone: 'bad', detail: signs.status });
  } else if (!signs.detector) {
    alerts.push({ text: 'Modelo de placas ausente', tone: 'warn', detail: signs.status });
  }

  return alerts;
}
