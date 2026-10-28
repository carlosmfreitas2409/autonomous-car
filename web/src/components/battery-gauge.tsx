import type { CartState } from '@/types';
import { readBattery } from '@/utils/battery';
import { cn } from '@/utils/cn';
import { formatNumber } from '@/utils/format';

interface BatteryGaugeProps {
  state: CartState | null;
  isStale: boolean;
}

const FILL = {
  ok: 'fill-accent',
  warn: 'fill-warn',
  bad: 'fill-white',
  none: 'fill-line-strong',
};
const MIN_LEVEL = 0.04;
const FILL_WIDTH = 26;

export function BatteryGauge({ state, isStale }: BatteryGaugeProps) {
  const battery = readBattery(state?.battery);
  const isEmpty = battery.tone === 'bad';
  const isWarning = battery.tone === 'warn';
  const level = battery.percent === null ? 0 : Math.max(MIN_LEVEL, battery.percent / 100);

  return (
    <div
      role="meter"
      aria-label="Bateria"
      aria-valuemin={0}
      aria-valuemax={100}
      aria-valuenow={battery.percent ?? undefined}
      title="Lê baixo enquanto os motores puxam; a leitura parada é a confiável."
      className={cn(
        'pointer-events-auto flex flex-none items-center gap-3 rounded-2xl pr-5 pl-4 transition-colors',
        isEmpty ? 'bg-stop text-white shadow-float' : 'float',
        isStale && 'opacity-60',
      )}
    >
      <svg aria-hidden width="40" height="20" viewBox="0 0 40 20" className="flex-none">
        <rect
          x="1"
          y="1"
          width="32"
          height="18"
          rx="4"
          strokeWidth={2}
          className={cn('fill-none', isEmpty ? 'stroke-white' : 'stroke-ink-3')}
        />
        <rect
          x="35"
          y="6"
          width="3"
          height="8"
          rx="1"
          className={isEmpty ? 'fill-white' : 'fill-ink-3'}
        />
        <rect
          x="4"
          y="4"
          width={FILL_WIDTH * level}
          height="12"
          rx="2"
          className={cn('transition-[width] duration-500', FILL[battery.tone])}
        />
      </svg>

      <div className="flex flex-col">
        <span
          className={cn(
            'text-lg/none font-semibold',
            isWarning && !battery.note && 'text-warn-ink',
          )}
        >
          {battery.percent ?? '—'}
          {battery.percent !== null && (
            <span
              className={cn('ml-0.5 text-xs font-medium', isEmpty ? 'text-white/80' : 'text-ink-3')}
            >
              %
            </span>
          )}
        </span>
        <span
          className={cn(
            'mt-1 text-xs/none whitespace-nowrap text-ink-3',
            isWarning && battery.note && 'font-medium text-warn-ink',
            isEmpty && 'font-semibold text-white',
          )}
        >
          {battery.note ?? `${formatNumber(battery.volts ?? 0, 1)} V`}
        </span>
      </div>
    </div>
  );
}
