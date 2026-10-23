import type { CartState } from '@/types';
import { cn } from '@/utils/cn';

import { type CargoTone, cargoReadouts, manifestText } from './utils/cargo-status';

interface CargoStatusProps {
  state: CartState;
}

const TONE: Record<CargoTone, string> = {
  normal: 'text-ink',
  muted: 'text-ink-3',
  warn: 'font-semibold text-warn-ink',
  stop: 'font-semibold text-stop-ink',
};

export function CargoStatus({ state }: CargoStatusProps) {
  const readouts = cargoReadouts(state.cargo);
  const manifest = manifestText(state.mission);

  return (
    <section aria-label="Compartimento" className="flex flex-col gap-2 border-t border-line pt-4">
      <div className="flex items-baseline justify-between gap-3">
        <h3 className="text-base font-semibold">Compartimento</h3>
        {readouts.length === 0 && <span className="text-sm text-ink-3">--</span>}
      </div>
      {readouts.length > 0 && (
        <p className="flex flex-wrap gap-x-1.5 text-sm">
          {readouts.map((readout, index) => (
            <span key={readout.text} className="flex gap-1.5">
              {index > 0 && <span className="text-ink-3">·</span>}
              <span
                title={readout.tone === 'muted' ? 'Estabilizando' : undefined}
                className={cn('tabular-nums', TONE[readout.tone])}
              >
                {readout.text}
              </span>
            </span>
          ))}
        </p>
      )}
      {manifest && <p className="text-sm text-ink-2">{manifest}</p>}
    </section>
  );
}
