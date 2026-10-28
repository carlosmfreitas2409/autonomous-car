import { driveAutonomous, stop } from '@/services/cart-commands';
import type { CartState } from '@/types';

import { Button } from '../ui/button';

interface SpeedCardProps {
  state: CartState;
}

function percent(value: number) {
  return `${Math.round(value * 100)} %`;
}

export function SpeedCard({ state }: SpeedCardProps) {
  const { speed } = state;
  const isStopped = state.drive.estopped;

  return (
    <>
      <div className="flex flex-col gap-1">
        <h2 className="text-xl font-semibold tracking-tight">Prova de velocidade</h2>
      </div>

      {speed && (
        <dl className="flex flex-col gap-2 text-base">
          <div className="flex justify-between gap-4">
            <dt className="text-ink-2">Velocidade máxima</dt>
            <dd className="font-semibold tabular-nums">{percent(speed.base)}</dd>
          </div>
          <div className="flex justify-between gap-4">
            <dt className="text-ink-2">Velocidade na curva</dt>
            <dd className="font-semibold tabular-nums">{percent(speed.min)}</dd>
          </div>
        </dl>
      )}

      <Button
        size="lg"
        variant={isStopped ? 'accent' : 'soft'}
        className="w-full"
        onClick={() => (isStopped ? driveAutonomous(state) : stop())}
      >
        {isStopped ? 'Iniciar volta' : 'Parar'}
      </Button>
    </>
  );
}
