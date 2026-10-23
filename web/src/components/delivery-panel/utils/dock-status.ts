import type { CartState } from '@/types';
import { formatClock } from '@/utils/format';
import { CHECK_FAILED_TEXT, CHECK_TEXT } from '@/utils/loop-text';

export type DockTone = 'normal' | 'warn' | 'stop';

export interface DockStatus {
  title: string;
  waited: string | null;
  line: string;
  tone: DockTone;
  hint: string | null;
  canWait: boolean;
}

const LID_RELEASES = 'Fechar a tampa libera o caminhão';

// The dashboard only reflects the release: the lid (or the tuner, or the
// PS5's Cross) gives it, never this page.
export function dockStatus(state: CartState): DockStatus | null {
  const { mission, cargo } = state;
  const dock = mission.dock;
  if (!dock) return null;
  const isStopped = state.drive.estopped;

  if (dock.released) {
    const check = mission.check;
    return {
      title: `Saindo de ${dock.letter}`,
      waited: null,
      line: check ? CHECK_TEXT[check] : 'Conferindo antes de sair',
      tone: check === 'blocked' ? 'stop' : check === 'lid' ? 'warn' : 'normal',
      hint: null,
      canWait: !isStopped,
    };
  }

  const isPickup = dock.role === 'pickup';
  const isFailed = !!dock.check_failed && !dock.override;
  let title = `Saindo de ${dock.letter}`;
  if (dock.role) title = `${isPickup ? 'Carregando' : 'Descarregando'} em ${dock.letter}`;

  let line = `Aguardando liberação para seguir até ${mission.target ?? mission.pickup}`;
  let tone: DockTone = 'normal';
  let hint: string | null = null;
  if (dock.role && isFailed && dock.check_failed) {
    line = CHECK_FAILED_TEXT[dock.check_failed];
    tone = 'stop';
    // Shut again, the lid releases without the weight check (mission.py).
    hint = isPickup ? 'Carregue e feche a tampa de novo' : 'Retire a carga e feche a tampa de novo';
  } else if (dock.role && cargo && !cargo.lid_closed) {
    line = 'Feche a tampa quando terminar';
    hint = LID_RELEASES;
  } else if (dock.role && cargo) {
    line = isPickup ? 'Abra, carregue e feche a tampa' : 'Abra, retire a carga e feche a tampa';
    hint = LID_RELEASES;
  } else if (dock.role) {
    line = isPickup ? 'Carregue e aguarde a liberação' : 'Retire a carga e aguarde a liberação';
  }
  if (isStopped) hint = 'Parado · retome para seguir';

  return {
    title,
    waited: `Esperando ${formatClock(state.t - dock.since)}`,
    line,
    tone,
    hint,
    canWait: false,
  };
}
