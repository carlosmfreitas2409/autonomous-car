import { PATH_NAMES } from '@/constants';
import { setPath } from '@/services/cart-commands';
import type { CartState, PathName } from '@/types';
import { cn } from '@/utils/cn';
import { formatNumber } from '@/utils/format';

import { Button } from './ui/button';
import { ButtonGroup } from './ui/panel';

interface PathControlProps {
  state: CartState;
  isFullWidth?: boolean;
}

const PATHS: PathName[] = ['normal', 'straight', 'detour'];

// DetourReturn's stages (path.py), as the loop names them.
function detourStageText(stage: string) {
  if (stage === 'waiting for the entrance') return 'Aguardando a entrada';
  const inside = /^inside ([\d.]+)s$/.exec(stage);
  if (inside) return `No desvio há ${formatNumber(Number(inside[1]), 1)} s`;
  const carried = /^hold carried (\d+)s$/.exec(stage);
  if (carried) return `Mantendo o perfil do desvio · ${carried[1]} s`;
  return stage;
}

export function PathControl({ state, isFullWidth = false }: PathControlProps) {
  const { path, detour_stage: detourStage } = state.path;

  return (
    <ButtonGroup
      aria-label="Trajeto"
      surface={isFullWidth ? 'inset' : 'floating'}
      className={cn(isFullWidth && 'w-full flex-wrap')}
    >
      {PATHS.map((name) => {
        const isCurrent = name === path;
        return (
          <Button
            key={name}
            aria-pressed={isCurrent}
            variant={isCurrent ? 'selected' : 'ghost'}
            className={cn('font-medium', !isCurrent && 'text-ink', isFullWidth && 'flex-1 px-0')}
            onClick={() => !isCurrent && setPath(name)}
          >
            {PATH_NAMES[name]}
          </Button>
        );
      })}

      {detourStage && (
        <span className="px-3 text-sm whitespace-nowrap text-ink-2 first-letter:uppercase">
          {detourStageText(detourStage)}
        </span>
      )}
    </ButtonGroup>
  );
}
