import { setSpeed } from '@/services/cart-commands';
import type { CartState } from '@/types';

import { Button } from '../ui/button';

import { Stepper } from './stepper';

interface SpeedSectionProps {
  state: CartState | null;
}

const STEP = 0.05;

function percent(value: number) {
  return Math.round(value * 100);
}

export function SpeedSection({ state }: SpeedSectionProps) {
  const speed = state?.speed ?? null;
  const isDisabled = !state || !speed;
  const isDefault =
    !speed || (speed.base === speed.defaults.base && speed.min === speed.defaults.min);

  return (
    <section aria-labelledby="speed-title" className="flex flex-col gap-2">
      <div className="flex h-8 items-center justify-between">
        <h3 id="speed-title" className="text-base font-semibold">
          Velocidade
        </h3>
        {state && speed && !isDefault && (
          <Button
            className="h-8 rounded-lg px-3 text-sm font-medium"
            onClick={() => setSpeed(speed.defaults)}
          >
            Restaurar {percent(speed.defaults.base)} / {percent(speed.defaults.min)} %
          </Button>
        )}
      </div>

      <div className="flex flex-col">
        <Stepper
          label="Velocidade máxima"
          value={speed?.base ?? null}
          step={STEP}
          max={1}
          decimals={0}
          scale={100}
          unit="%"
          isDisabled={isDisabled}
          onChange={(base) => state && speed && setSpeed({ base, min: Math.min(speed.min, base) })}
        />
        <Stepper
          label="Velocidade na curva"
          value={speed?.min ?? null}
          step={STEP}
          max={speed?.base ?? 1}
          decimals={0}
          scale={100}
          unit="%"
          isDisabled={isDisabled}
          onChange={(min) => state && setSpeed({ min })}
        />
      </div>
      <p className="text-sm text-ink-2">A velocidade máxima também limita o modo manual.</p>
    </section>
  );
}
