import type { Battery } from '@/types';

import { formatNumber } from './format';
import type { Tone } from './health';

const LOW_PERCENT = 15;
const WARN_PERCENT = 30;
const SILENT_S = 5;

export interface BatteryReading {
  percent: number | null;
  volts: number | null;
  tone: Tone;
  note: string | null;
}

export function readBattery(battery: Battery | null | undefined): BatteryReading {
  if (!battery) return { percent: null, volts: null, tone: 'none', note: 'Sem leitura' };

  const { percent, volts, age_s: ageS } = battery;
  if (ageS > SILENT_S) {
    return { percent, volts, tone: 'warn', note: `Sem leitura há ${formatNumber(ageS)} s` };
  }
  if (percent <= LOW_PERCENT) return { percent, volts, tone: 'bad', note: 'Carregue logo' };
  return { percent, volts, tone: percent <= WARN_PERCENT ? 'warn' : 'ok', note: null };
}
