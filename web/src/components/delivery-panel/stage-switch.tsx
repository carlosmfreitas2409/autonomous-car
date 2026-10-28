import { CHALLENGE_NAMES } from '@/constants';
import { setChallenge } from '@/services/cart-commands';
import type { CartState, Challenge } from '@/types';

import { Button } from '../ui/button';
import { ButtonGroup } from '../ui/panel';

interface StageSwitchProps {
  state: CartState;
}

const CHALLENGES: Challenge[] = ['delivery', 'speed'];

export function StageSwitch({ state }: StageSwitchProps) {
  const isLocked = !state.drive.estopped;

  return (
    <ButtonGroup
      aria-label="Prova"
      surface="inset"
      title={isLocked ? 'Pare o caminhão para trocar a prova' : undefined}
      className="w-full flex-none"
    >
      {CHALLENGES.map((challenge) => {
        const isCurrent = challenge === state.challenge;
        return (
          <Button
            key={challenge}
            aria-pressed={isCurrent}
            variant={isCurrent ? 'selected' : 'ghost'}
            disabled={isLocked && !isCurrent}
            className="flex-1 px-0"
            onClick={() => !isCurrent && setChallenge(challenge)}
          >
            {CHALLENGE_NAMES[challenge]}
          </Button>
        );
      })}
    </ButtonGroup>
  );
}
