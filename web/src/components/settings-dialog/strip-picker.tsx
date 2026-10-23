import { STRIP_COLOURS } from '@/constants';
import { setLights } from '@/services/cart-commands';
import type { CartState, LightSettings, StripEffect } from '@/types';
import { cn } from '@/utils/cn';

import { Button } from '../ui/button';

interface StripPickerProps {
  state: CartState | null;
  settings: LightSettings | null;
}

interface SwatchProps {
  effect: LightSettings['strip'];
}

const EFFECTS: { id: LightSettings['strip']; label: string }[] = [
  { id: 'auto', label: 'Auto' },
  { id: 'off', label: 'Desligada' },
  { id: 'red', label: 'Vermelha' },
  { id: 'green', label: 'Verde' },
  { id: 'blue', label: 'Azul' },
  { id: 'cyan', label: 'Turquesa' },
  { id: 'red_pulse', label: 'Pulso' },
  { id: 'strobe', label: 'Estrobo' },
  { id: 'rainbow', label: 'Arco-íris' },
  { id: 'police', label: 'Polícia' },
];

const PATTERNS: Partial<Record<StripEffect, string>> = {
  rainbow: 'conic-gradient(#ff3b30, #ffcc00, #34c759, #22c8e5, #2f6dff, #b45cff, #ff3b30)',
  police: `linear-gradient(90deg, ${STRIP_COLOURS.red} 50%, ${STRIP_COLOURS.blue} 50%)`,
};

function Swatch({ effect }: SwatchProps) {
  if (effect === 'auto' || effect === 'off') {
    return (
      <span
        aria-hidden
        className={cn(
          'size-4 rounded-full border-2 border-current opacity-60',
          effect === 'auto' && 'border-dashed',
        )}
      />
    );
  }
  return (
    <span
      aria-hidden
      className="size-4 rounded-full ring-1 ring-black/15"
      style={{ background: PATTERNS[effect] ?? STRIP_COLOURS[effect] ?? undefined }}
    />
  );
}

export function StripPicker({ state, settings }: StripPickerProps) {
  const isDisabled = !state || !settings;

  return (
    <section aria-labelledby="strip-title" className="@container flex flex-col gap-2">
      <h3 id="strip-title" className="text-base font-semibold">
        Fita de LED
      </h3>
      <div role="group" className="grid grid-cols-5 gap-1 rounded-2xl bg-fill p-1">
        {EFFECTS.map((effect) => {
          const isSelected = settings?.strip === effect.id;
          return (
            <Button
              key={effect.id}
              aria-pressed={isSelected}
              variant={isSelected ? 'selected' : 'ghost'}
              disabled={isDisabled}
              className="h-16 flex-col gap-2 rounded-xl px-0 text-xs font-medium @sm:text-sm"
              onClick={() => state && !isSelected && setLights(state, { strip: effect.id })}
            >
              <Swatch effect={effect.id} />
              {effect.label}
            </Button>
          );
        })}
      </div>
      <p className="text-sm text-ink-2">
        Auto: turquesa no autônomo, verde no manual, vermelha parado, azul na doca.
      </p>
    </section>
  );
}
