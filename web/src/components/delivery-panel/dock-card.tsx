import { holdDeparture } from '@/services/cart-commands';
import type { CartState } from '@/types';
import { cn } from '@/utils/cn';

import { Button } from '../ui/button';

import { type DockTone, dockStatus } from './utils/dock-status';

interface DockCardProps {
  state: CartState;
}

const LINE: Record<DockTone, string> = {
  normal: 'text-ink',
  warn: 'text-warn-ink',
  stop: 'text-stop-ink',
};

export function DockCard({ state }: DockCardProps) {
  const status = dockStatus(state);
  if (!status) return null;

  return (
    <section aria-label="Doca" className="flex flex-col gap-3">
      <div className="flex flex-col gap-1">
        <h2 className="text-2xl font-semibold tracking-tight">{status.title}</h2>
        {status.waited && <span className="text-sm text-ink-2 tabular-nums">{status.waited}</span>}
      </div>
      <div className="flex flex-col gap-1">
        <p className={cn('text-lg font-semibold text-balance', LINE[status.tone])}>{status.line}</p>
        {status.hint && <p className="text-sm text-ink-2">{status.hint}</p>}
      </div>
      {status.canWait && (
        <Button size="xl" variant="soft" className="w-full" onClick={() => holdDeparture()}>
          Esperar
        </Button>
      )}
    </section>
  );
}
