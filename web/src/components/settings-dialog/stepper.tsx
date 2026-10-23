import { useId, useState } from 'react';

import { cn } from '@/utils/cn';

import { Button } from '../ui/button';
import { ButtonGroup } from '../ui/panel';

interface StepperProps {
  label: string;
  value: number | null;
  step: number;
  max: number;
  decimals: number;
  scale?: number;
  unit?: string;
  isDisabled: boolean;
  onChange: (value: number) => void;
}

const PRECISION = 1e5;

function bounded(value: number, max: number) {
  return Math.min(max, Math.max(0, Math.round(value * PRECISION) / PRECISION));
}

export function Stepper({
  label,
  value,
  step,
  max,
  decimals,
  scale = 1,
  unit,
  isDisabled,
  onChange,
}: StepperProps) {
  const id = useId();
  const [draft, setDraft] = useState<string | null>(null);
  const shown = value === null ? '' : (value * scale).toFixed(decimals).replace('.', ',');
  const name = label.toLowerCase();

  function handleCommit() {
    if (draft === null) return;
    const typed = Number.parseFloat(draft.replace(',', '.'));
    setDraft(null);
    if (value !== null && Number.isFinite(typed)) {
      const next = bounded(typed / scale, max);
      if (next !== value) onChange(next);
    }
  }

  function handleKeyDown(event: React.KeyboardEvent<HTMLInputElement>) {
    if (event.key === 'Enter') event.currentTarget.blur();
    if (event.key === 'Escape') {
      event.preventDefault();
      setDraft(null);
      event.currentTarget.blur();
    }
  }

  return (
    <div className="flex h-12 items-center justify-between gap-4">
      <label htmlFor={id} className="text-md">
        {label}
      </label>
      <ButtonGroup surface="inset">
        <Button
          size="icon"
          aria-label={`Diminuir ${name}`}
          disabled={isDisabled || value === null || value <= 0}
          className="rounded-xl text-lg"
          onClick={() => value !== null && onChange(bounded(value - step, max))}
        >
          −
        </Button>
        <span className="flex items-baseline">
          <input
            id={id}
            inputMode="decimal"
            autoComplete="off"
            disabled={isDisabled || value === null}
            value={draft ?? shown}
            className={cn(
              'h-8 rounded-lg bg-transparent text-center text-md font-semibold outline-none focus:bg-white focus:shadow-float disabled:text-ink-3',
              unit ? 'w-10' : 'w-16',
            )}
            onFocus={(event) => {
              setDraft(shown);
              event.currentTarget.select();
            }}
            onChange={(event) => setDraft(event.target.value)}
            onBlur={handleCommit}
            onKeyDown={handleKeyDown}
          />
          {unit && <span className="pr-2 text-xs font-medium text-ink-3">{unit}</span>}
        </span>
        <Button
          size="icon"
          aria-label={`Aumentar ${name}`}
          disabled={isDisabled || value === null || value >= max}
          className="rounded-xl text-lg"
          onClick={() => value !== null && onChange(bounded(value + step, max))}
        >
          +
        </Button>
      </ButtonGroup>
    </div>
  );
}
