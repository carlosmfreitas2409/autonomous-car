import { cn } from '@/utils/cn';
import { type HealthInput, type Tone, checkHealth, summarizeHealth } from '@/utils/health';

interface HealthPanelProps {
  link: HealthInput;
  isBare?: boolean;
}

const DOT: Record<Tone, string> = {
  ok: 'bg-accent',
  warn: 'bg-warn',
  bad: 'bg-stop',
  none: 'bg-line-strong',
};

const SUMMARY: Record<Tone, string> = {
  ok: 'text-accent-ink',
  warn: 'text-warn-ink',
  bad: 'text-stop-ink',
  none: 'text-ink-2',
};

const VALUE: Record<Tone, string> = {
  ok: 'text-ink-2',
  none: 'text-ink-2',
  warn: 'font-medium text-warn-ink',
  bad: 'font-semibold text-stop-ink',
};

export function HealthPanel({ link, isBare = false }: HealthPanelProps) {
  const health = checkHealth(link);
  const summary = summarizeHealth(health);

  return (
    <section
      aria-label="Sistema"
      className={cn(
        'w-full',
        !isBare && 'pointer-events-auto w-64 flex-none rounded-2xl px-4 py-3 float',
      )}
    >
      <div className="mb-1 flex flex-wrap items-baseline justify-between gap-x-3">
        <h2 className="text-base font-semibold">Sistema</h2>
        <span className={cn('text-sm font-semibold whitespace-nowrap', SUMMARY[summary.tone])}>
          {summary.text}
        </span>
      </div>

      <dl>
        {health.checks.map((check) => (
          <div key={check.label} className="flex items-center gap-2 py-1 text-sm">
            <span aria-hidden className={cn('size-2 flex-none rounded-full', DOT[check.tone])} />
            <dt className="shrink-0">{check.label}</dt>
            <dd
              title={check.value}
              className={cn('ml-auto truncate first-letter:uppercase', VALUE[check.tone])}
            >
              {check.value}
            </dd>
          </div>
        ))}
      </dl>
    </section>
  );
}
