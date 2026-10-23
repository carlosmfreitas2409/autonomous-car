import { useState } from 'react';

import type { CartState, Challenge } from '@/types';
import { cn } from '@/utils/cn';

import { Button } from '../ui/button';
import { Dialog } from '../ui/dialog';

import { CompartmentSection } from './compartment-section';
import { LightsSection } from './lights-section';
import { PidSection } from './pid-section';
import { SpeedSection } from './speed-section';
import { StripPicker } from './strip-picker';

const STAGE_TUNING: Record<Challenge, string> = {
  delivery: 'Velocidade e PID da prova de entrega',
  speed: 'Velocidade e PID da prova de velocidade',
};

interface SettingsButtonProps {
  state: CartState | null;
  className?: string;
}

export function SettingsButton({ state, className }: SettingsButtonProps) {
  const [isOpen, setIsOpen] = useState(false);
  const settings = state?.lights?.settings ?? null;

  return (
    <>
      <Button
        aria-label="Ajustes"
        aria-haspopup="dialog"
        title="Ajustes"
        className={cn('pointer-events-auto size-12 flex-none rounded-2xl px-0 float', className)}
        onClick={() => setIsOpen(true)}
      >
        <svg
          aria-hidden
          width="20"
          height="20"
          viewBox="0 0 20 20"
          fill="none"
          stroke="currentColor"
          strokeWidth="1.75"
          strokeLinecap="round"
        >
          <path d="M3 6h8M15 6h2M3 14h2M9 14h8" />
          <circle cx="13" cy="6" r="2" />
          <circle cx="7" cy="14" r="2" />
        </svg>
      </Button>

      <Dialog isOpen={isOpen} aria-labelledby="settings-title" onClose={() => setIsOpen(false)}>
        <div className="flex max-h-[calc(100dvh-32px)] w-[min(880px,calc(100vw-32px))] flex-col gap-6 overflow-auto rounded-3xl bg-panel p-6 shadow-raised">
          <header className="flex items-center justify-between gap-4">
            <div className="flex flex-col gap-1">
              <h2 id="settings-title" className="text-xl font-semibold tracking-tight">
                Ajustes
              </h2>
              {state?.challenge && (
                <p className="text-sm text-ink-2">
                  {STAGE_TUNING[state.challenge]}. Cada prova guarda os seus.
                </p>
              )}
            </div>
            <Button size="icon" aria-label="Fechar" onClick={() => setIsOpen(false)}>
              <svg
                aria-hidden
                width="14"
                height="14"
                viewBox="0 0 14 14"
                stroke="currentColor"
                strokeWidth="1.75"
                strokeLinecap="round"
              >
                <path d="M2 2l10 10M12 2 2 12" />
              </svg>
            </Button>
          </header>

          {state && (!settings || !state.speed || !state.pid) && (
            <p className="text-sm text-warn-ink">
              Este caminhão não informa os ajustes. Atualize o software do carro.
            </p>
          )}

          <div className="@container">
            <div className="grid gap-6 @3xl:grid-cols-2 @3xl:gap-x-10">
              <div className="flex flex-col gap-6">
                <SpeedSection state={state} />
                <PidSection state={state} />
                <CompartmentSection state={state} />
              </div>
              <div className="flex flex-col gap-6">
                <LightsSection state={state} settings={settings} />
                <StripPicker state={state} settings={settings} />
              </div>
            </div>
          </div>
        </div>
      </Dialog>
    </>
  );
}
