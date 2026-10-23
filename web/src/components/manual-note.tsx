import type { CartState } from '@/types';
import { manualNoteText } from '@/utils/loop-text';

import { Panel } from './ui/panel';

interface ManualNoteProps {
  state: CartState;
  gamepad: string | null;
  isController: boolean;
}

function manualProblem(state: CartState, isController: boolean) {
  if (!isController) return 'Outro console está dirigindo';
  if (state.drive.lease_lost) return 'Conexão perdida · parado';
  return state.drive.manual_note ? manualNoteText(state.drive.manual_note) : '';
}

export function ManualNote({ state, gamepad, isController }: ManualNoteProps) {
  const problem = manualProblem(state, isController);

  return (
    <Panel className="flex h-12 items-center gap-2 rounded-2xl px-4 text-sm text-ink-2">
      <span className="truncate">
        {gamepad ? `Controle: ${gamepad}` : 'Conecte um controle para dirigir'}
      </span>
      {problem && (
        <span className="ml-auto shrink-0 font-semibold text-stop-ink first-letter:uppercase">
          {problem}
        </span>
      )}
    </Panel>
  );
}
