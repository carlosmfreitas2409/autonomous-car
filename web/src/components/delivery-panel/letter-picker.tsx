import type { CartState, Letter } from '@/types';
import { setLetters } from '@/services/cart-commands';

import { Button } from '../ui/button';

interface LetterPickerProps {
  kind: 'pickup' | 'delivery';
  state: CartState;
}

const LETTERS: Letter[] = ['A', 'B', 'C'];

export function LetterPicker({ kind, state }: LetterPickerProps) {
  const mission = state.mission;
  const selected = mission[kind];
  const isPickup = kind === 'pickup';

  function handlePick(letter: Letter) {
    const next = { pickup: mission.pickup, delivery: mission.delivery };
    next[kind] = selected === letter ? null : letter;
    setLetters(state, next.pickup, next.delivery);
  }

  return (
    <div
      role="group"
      aria-label={isPickup ? 'Coleta' : 'Entrega'}
      className="flex items-center gap-2"
    >
      <span className="w-10 text-sm text-ink-2">{isPickup ? 'De' : 'Para'}</span>
      {LETTERS.map((letter) => {
        const isSelected = selected === letter;
        let variant: 'soft' | 'selected' | 'accent' = 'soft';
        if (isSelected) variant = isPickup ? 'selected' : 'accent';
        return (
          <Button
            key={letter}
            aria-pressed={isSelected}
            variant={variant}
            className="flex-1 rounded-lg px-0"
            onClick={() => handlePick(letter)}
          >
            {letter}
          </Button>
        );
      })}
    </div>
  );
}
