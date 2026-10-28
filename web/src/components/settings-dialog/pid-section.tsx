import { setPid } from '@/services/cart-commands';
import type { CartState, PidGain } from '@/types';

import { Button } from '../ui/button';

import { Stepper } from './stepper';

interface PidSectionProps {
  state: CartState | null;
}

const GAINS: { id: PidGain; label: string; step: number }[] = [
  { id: 'kp', label: 'Kp', step: 0.001 },
  { id: 'ki', label: 'Ki', step: 0.0005 },
  { id: 'kd', label: 'Kd', step: 0.0005 },
];

export function PidSection({ state }: PidSectionProps) {
  const pid = state?.pid ?? null;
  const isDisabled = !state || !pid;
  const isDefault = !pid || GAINS.every((gain) => pid[gain.id] === pid.defaults[gain.id]);

  return (
    <section aria-labelledby="pid-title" className="flex flex-col gap-2">
      <div className="flex h-8 items-center justify-between">
        <h3 id="pid-title" className="text-base font-semibold">
          PID da direção
        </h3>
        {state && pid && !isDefault && (
          <Button
            className="h-8 rounded-lg px-3 text-sm font-medium"
            onClick={() => setPid(pid.defaults)}
          >
            Restaurar padrão
          </Button>
        )}
      </div>

      <div className="flex flex-col">
        {GAINS.map((gain) => (
          <Stepper
            key={gain.id}
            label={gain.label}
            value={pid?.[gain.id] ?? null}
            step={gain.step}
            max={pid?.max[gain.id] ?? 0}
            decimals={4}
            isDisabled={isDisabled}
            onChange={(value) => state && setPid({ [gain.id]: value })}
          />
        ))}
      </div>
    </section>
  );
}
