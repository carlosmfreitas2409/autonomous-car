import { setLights } from '@/services/cart-commands';
import type { CartState, LightSettings, TurnSignal } from '@/types';

import { Button } from '../ui/button';
import { ButtonGroup } from '../ui/panel';
import { Switch } from '../ui/switch';

interface LightsSectionProps {
  state: CartState | null;
  settings: LightSettings | null;
}

type LampKey = 'drl' | 'low_beam' | 'logo' | 'projector';

const SIGNALS: { id: TurnSignal; label: string }[] = [
  { id: 'off', label: 'Desligada' },
  { id: 'left', label: 'Esquerda' },
  { id: 'right', label: 'Direita' },
  { id: 'hazard', label: 'Alerta' },
];

const LAMPS: { id: LampKey; label: string }[] = [
  { id: 'drl', label: 'Luz diurna' },
  { id: 'low_beam', label: 'Farol baixo' },
  { id: 'logo', label: 'Projetor do logo' },
  { id: 'projector', label: 'Projetor da porta' },
];

function overrideNote(shown: TurnSignal) {
  if (shown === 'off') return 'Por enquanto o carro está com a seta desligada.';
  if (shown === 'hazard') return 'Por enquanto o carro está com o alerta ligado.';
  return `Por enquanto o carro está dando seta para a ${shown === 'left' ? 'esquerda' : 'direita'}.`;
}

export function LightsSection({ state, settings }: LightsSectionProps) {
  const isDisabled = !state || !settings;
  const shown = state?.lights?.signal;
  const isOverridden = !!settings && !!shown && shown !== settings.turn_signal;

  return (
    <section aria-labelledby="lights-title" className="@container flex flex-col gap-2">
      <h3 id="lights-title" className="text-base font-semibold">
        Luzes
      </h3>

      <div className="flex flex-col">
        <div className="flex flex-col gap-2 py-2 @sm:h-12 @sm:flex-row @sm:items-center @sm:justify-between @sm:gap-4 @sm:py-0">
          <span id="signal-label" className="text-md">
            Seta
          </span>
          <ButtonGroup surface="inset" aria-labelledby="signal-label" className="@max-sm:w-full">
            {SIGNALS.map((signal) => {
              const isSelected = settings?.turn_signal === signal.id;
              return (
                <Button
                  key={signal.id}
                  aria-pressed={isSelected}
                  variant={isSelected ? 'selected' : 'ghost'}
                  disabled={isDisabled}
                  className="h-8 rounded-xl px-3 text-sm font-medium @max-sm:flex-1 @max-sm:px-0"
                  onClick={() =>
                    state && !isSelected && setLights(state, { turn_signal: signal.id })
                  }
                >
                  {signal.label}
                </Button>
              );
            })}
          </ButtonGroup>
        </div>
        {isOverridden && <p className="pb-2 text-sm text-ink-2">{overrideNote(shown)}</p>}

        {LAMPS.map((lamp) => (
          <Switch
            key={lamp.id}
            isChecked={!!settings?.[lamp.id]}
            disabled={isDisabled}
            onCheckedChange={(isChecked) => state && setLights(state, { [lamp.id]: isChecked })}
          >
            {lamp.label}
          </Switch>
        ))}
      </div>
    </section>
  );
}
