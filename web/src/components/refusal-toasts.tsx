import type { TimedReply } from '@/hooks/use-control';
import { useRefusals } from '@/hooks/use-refusals';
import { cn } from '@/utils/cn';

import { Mark } from './ui/mark';

interface RefusalToastsProps {
  replies: TimedReply[];
  isCompact: boolean;
}

export function RefusalToasts({ replies, isCompact }: RefusalToastsProps) {
  const refusals = useRefusals(replies);

  return (
    <div
      className={cn(
        'absolute left-1/2 z-20 flex -translate-x-1/2 flex-col items-center gap-2',
        isCompact ? 'bottom-40' : 'bottom-24',
      )}
    >
      {refusals.map((refusal) => (
        <div
          key={refusal.id}
          className="flex items-center gap-2.5 rounded-xl bg-ink px-4 py-2 text-sm font-medium text-white"
        >
          <Mark tone="stop" className="text-stop" />
          {refusal.text}
        </div>
      ))}
    </div>
  );
}
