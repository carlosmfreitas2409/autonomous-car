import type { CartState, LightState } from '@/types';
import { cn } from '@/utils/cn';
import { formatNumber } from '@/utils/format';
import { isActionableSign } from '@/utils/signs';

import { Panel } from './ui/panel';

interface TrafficLightChipProps {
  state: CartState;
}

const LIGHT_NAMES: Record<LightState, string> = {
  red: 'Sinal vermelho',
  yellow: 'Sinal amarelo',
  green: 'Sinal verde',
  unknown: 'Semáforo',
};

const LAMP: Record<LightState, string> = {
  red: 'bg-stop',
  yellow: 'bg-warn',
  green: 'bg-ok',
  unknown: 'bg-ink-3',
};

export function TrafficLightChip({ state }: TrafficLightChipProps) {
  const nearest = state.signs.tracks
    .filter((track) => track.label === 'traffic_light' && isActionableSign(track))
    .sort((a, b) => (a.distance_m ?? Infinity) - (b.distance_m ?? Infinity))[0];
  if (!nearest) return null;

  return (
    <Panel className="flex items-center gap-2.5 self-start rounded-full py-2 pr-4 pl-2.5 text-base font-medium">
      <span aria-hidden className={cn('size-3 rounded-full', LAMP[nearest.light])} />
      <span>
        {LIGHT_NAMES[nearest.light]}
        {nearest.distance_m !== null && ` · ${formatNumber(nearest.distance_m, 1)} m`}
      </span>
    </Panel>
  );
}
