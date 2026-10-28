import type { CartState } from '@/types';
import { cn } from '@/utils/cn';
import { formatNumber } from '@/utils/format';

interface DriveReadoutsProps {
  state: CartState;
  isCompact?: boolean;
}

const WHEEL_DEG_PER_STEER = 120;

export function DriveReadouts({ state, isCompact = false }: DriveReadoutsProps) {
  const { applied_throttle: output, steering, fw } = state.drive;
  const sign = steering >= 0 ? '+' : '−';

  return (
    <div className="flex items-end gap-8">
      <div>
        <div className="flex items-baseline gap-0.5">
          <span
            className={cn(
              'font-medium tracking-tighter',
              isCompact ? 'text-6xl/none' : 'text-readout',
            )}
          >
            {Math.round(Math.abs(output) * 100)}
          </span>
          <span className="text-2xl font-medium text-ink-3">%{output < -0.01 && ' ré'}</span>
        </div>
        <div className="mt-2 text-base text-ink-2">Acelerador{!fw && ' (comandado)'}</div>
      </div>

      <div className="flex items-center gap-2.5 pb-6">
        <svg
          aria-hidden
          width="32"
          height="32"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth={1.8}
          className="transition-transform duration-200"
          style={{ rotate: `${steering * WHEEL_DEG_PER_STEER}deg` }}
        >
          <circle cx="12" cy="12" r="9.5" />
          <circle cx="12" cy="12" r="2.2" />
          <path d="M12 14.2V21.5M9.9 11.4 2.8 9.8M14.1 11.4l7.1-1.6" />
        </svg>
        <span className="text-2xl font-medium" title="Direção">
          {sign}
          {formatNumber(Math.abs(steering), 2)}
        </span>
      </div>
    </div>
  );
}
