import { cn } from '@/utils/cn';

interface MarkProps {
  tone: 'stop' | 'warn' | 'idle';
  className?: string;
}

function Mark({ tone, className }: MarkProps) {
  return (
    <span
      aria-hidden
      className={cn(
        'inline-block shrink-0 bg-current',
        tone === 'stop' && 'size-2 rounded-xs',
        tone === 'warn' && 'h-2 w-2.5 [clip-path:polygon(50%_0,100%_100%,0_100%)]',
        tone === 'idle' && 'size-2 rounded-full',
        className,
      )}
    />
  );
}

export { Mark };
