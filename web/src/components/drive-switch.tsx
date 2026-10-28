import { driveAutonomous, driveManual, resume, stop } from '@/services/cart-commands';
import type { CartState } from '@/types';
import { cn } from '@/utils/cn';

import { Button } from './ui/button';
import { ButtonGroup } from './ui/panel';

interface DriveSwitchProps {
  state: CartState | null;
  isConnected: boolean;
  isFullWidth?: boolean;
}

export function DriveSwitch({ state, isConnected, isFullWidth = false }: DriveSwitchProps) {
  const mode = state?.drive.mode;
  const isStopped = !!state?.drive.estopped;
  const segment = cn(isFullWidth && 'flex-1 px-0');

  function handleStopOrResume() {
    if (isStopped) resume();
    else stop();
  }

  return (
    <ButtonGroup aria-label="Direção" className={cn('flex-none', isFullWidth && 'w-full')}>
      <Button
        aria-pressed={mode === 'autonomous'}
        variant={mode === 'autonomous' ? 'selected' : 'ghost'}
        disabled={!state}
        className={segment}
        onClick={() => state && driveAutonomous(state)}
      >
        Autônomo
      </Button>

      <Button
        aria-pressed={mode === 'manual'}
        variant={mode === 'manual' ? 'selected' : 'ghost'}
        disabled={!state}
        className={segment}
        onClick={() => state && driveManual(state)}
      >
        Manual
      </Button>

      <Button
        variant={isStopped ? 'selected' : 'stop'}
        disabled={isStopped ? !state : !isConnected}
        className={cn('px-4 disabled:opacity-50', segment)}
        onClick={handleStopOrResume}
      >
        {isStopped ? (
          <svg aria-hidden width="10" height="10" viewBox="0 0 10 10" className="fill-current">
            <path d="M1 0.5 9.5 5 1 9.5Z" />
          </svg>
        ) : (
          <span aria-hidden className="size-2.5 rounded-xs bg-current" />
        )}
        {isStopped ? 'Retomar' : 'Parar'}
        {!isStopped && !isFullWidth && (
          <kbd className="rounded border border-current px-1 py-0.5 font-sans text-2xs leading-none opacity-75">
            Espaço
          </kbd>
        )}
      </Button>
    </ButtonGroup>
  );
}
