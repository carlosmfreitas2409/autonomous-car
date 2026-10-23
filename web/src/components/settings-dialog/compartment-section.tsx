import { openHint } from '@/components/delivery-panel/utils/cargo-status';
import { openCompartment } from '@/services/cart-commands';
import type { CartState } from '@/types';

import { Button } from '../ui/button';

interface CompartmentSectionProps {
  state: CartState | null;
}

export function CompartmentSection({ state }: CompartmentSectionProps) {
  const hint = state ? openHint(state) : 'Sem conexão com o caminhão';

  return (
    <section aria-labelledby="compartment-title" className="flex flex-col gap-2">
      <h3 id="compartment-title" className="text-base font-semibold">
        Compartimento
      </h3>
      <div className="flex h-12 items-center justify-between gap-4">
        <span className="text-md">{hint ?? 'Destrava até a tampa abrir, por até 10 s'}</span>
        <Button
          variant="soft"
          disabled={hint !== null}
          className="flex-none disabled:text-ink-3/70"
          onClick={() => state && openCompartment(state)}
        >
          Abrir
        </Button>
      </div>
    </section>
  );
}
