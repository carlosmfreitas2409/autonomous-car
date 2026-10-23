import type { CartState } from '@/types';
import { worldAlerts } from '@/utils/alerts';
import { cn } from '@/utils/cn';

import { Mark } from './ui/mark';

interface AlertListProps {
  state: CartState;
  max?: number;
}

export function AlertList({ state, max }: AlertListProps) {
  const all = worldAlerts(state);
  if (all.length === 0) return null;
  const alerts = max === undefined ? all : all.slice(0, max);

  return (
    <div className="flex flex-col items-start gap-2">
      {alerts.map((alert, i) => (
        <div
          key={i}
          className={cn(
            'flex items-center gap-2 rounded-full px-4 py-2 text-base font-medium',
            alert.tone === 'bad' ? 'bg-stop-soft text-stop-ink' : 'bg-warn-soft text-warn-ink',
          )}
        >
          <Mark tone={alert.tone === 'bad' ? 'stop' : 'warn'} />
          <span className="first-letter:uppercase">
            {alert.text}
            {alert.detail && <span className="font-normal opacity-80"> · {alert.detail}</span>}
          </span>
        </div>
      ))}
      {alerts.length < all.length && (
        <div className="px-4 text-sm font-medium text-ink-2">
          +{all.length - alerts.length} {all.length - alerts.length === 1 ? 'aviso' : 'avisos'}
        </div>
      )}
    </div>
  );
}
