import type { CartState } from '@/types';
import { cn } from '@/utils/cn';
import { stopTitle } from '@/utils/loop-text';

interface StatusPillProps {
  state: CartState | null;
  isConnected: boolean;
  className?: string;
}

type Tone = 'stop' | 'autonomous' | 'manual' | 'idle';

const DOT: Record<Tone, string> = {
  stop: 'bg-white',
  autonomous: 'bg-accent',
  manual: 'bg-warn',
  idle: 'bg-line-strong',
};

interface Status {
  title: string;
  hint: string;
  tone: Tone;
}

function describe(state: CartState | null, isConnected: boolean): Status {
  const drive = state?.drive;
  if (!state || !drive) {
    const title = isConnected ? 'Aguardando o caminhão' : 'Sem conexão com o caminhão';
    return { title, hint: '', tone: 'idle' };
  }
  if (drive.estopped) return { title: 'Parado', hint: 'Retome para seguir', tone: 'stop' };
  const stopped = stopTitle(state);
  if (stopped) return { title: stopped, hint: '', tone: 'stop' };
  if (drive.mode === 'manual') return { title: 'Manual', hint: '', tone: 'manual' };
  if (state.challenge === 'speed') {
    return { title: 'Autônomo', hint: 'prova de velocidade', tone: 'autonomous' };
  }

  const { stage, pickup, delivery } = state.mission;
  if (stage === 'pickup' || stage === 'delivery') {
    return { title: 'Em entrega', hint: `${pickup} → ${delivery}`, tone: 'autonomous' };
  }
  return { title: 'Autônomo', hint: '', tone: 'autonomous' };
}

export function StatusPill({ state, isConnected, className }: StatusPillProps) {
  const { title, hint, tone } = describe(state, isConnected);

  return (
    <div
      data-status
      className={cn(
        'flex h-12 max-w-full items-center gap-2.5 self-start rounded-full px-5 whitespace-nowrap transition-colors',
        tone === 'stop' ? 'bg-stop text-white shadow-float' : 'float',
        className,
      )}
    >
      <span aria-hidden className={cn('size-2 shrink-0 rounded-full', DOT[tone])} />
      <span
        title={title}
        className={cn(
          'text-lg font-semibold first-letter:uppercase',
          hint ? 'shrink-0' : 'truncate',
          tone === 'idle' && 'text-ink-3',
        )}
      >
        {title}
      </span>
      {hint && <span className="min-w-0 truncate text-md opacity-70">{hint}</span>}
    </div>
  );
}
