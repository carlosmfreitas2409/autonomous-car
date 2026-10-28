import type { CartState, Mission, TrackLayout } from '@/types';
import { cancelDelivery, driveAutonomous, stop } from '@/services/cart-commands';
import { cn } from '@/utils/cn';

import { Button } from '../ui/button';

import { CargoStatus } from './cargo-status';
import { DockCard } from './dock-card';
import { LetterPicker } from './letter-picker';
import { SpeedCard } from './speed-card';
import { StageSwitch } from './stage-switch';
import { TrackMap } from './track-map';
import { TurnIcon } from './turn-icon';
import {
  type DeliveryAction,
  type DeliveryStatus,
  deliveryStatus,
  positionText,
} from './utils/delivery-status';

interface DeliveryPanelProps {
  state: CartState | null;
  layout: TrackLayout | null;
  isStale: boolean;
  isBare?: boolean;
}

interface DeliveryHeaderProps {
  mission: Mission;
  status: DeliveryStatus;
}

interface DeliveryCardProps {
  isBare: boolean;
  label?: string;
  className?: string;
  children: React.ReactNode;
}

function DeliveryCard({ isBare, label = 'Entrega', className, children }: DeliveryCardProps) {
  return (
    <section
      aria-label={label}
      className={cn(
        isBare
          ? 'flex w-full flex-col gap-4'
          : 'pointer-events-auto mt-auto flex min-h-0 w-[min(304px,28vw)] flex-initial flex-col gap-4 overflow-auto rounded-3xl p-5 float-raised',
        className,
      )}
    >
      {children}
    </section>
  );
}

export function DeliveryPanel({ state, layout, isStale, isBare = false }: DeliveryPanelProps) {
  if (!state) {
    return (
      <DeliveryCard isBare={isBare}>
        <h2 className="text-xl font-semibold tracking-tight text-ink-3">Entrega</h2>
        {layout && <TrackMap state={null} layout={layout} />}
      </DeliveryCard>
    );
  }

  const stageSwitch = state.challenge && <StageSwitch state={state} />;

  if (state.challenge === 'speed') {
    return (
      <DeliveryCard
        isBare={isBare}
        label="Prova de velocidade"
        className={cn(isStale && 'opacity-50 grayscale')}
      >
        {stageSwitch}
        <SpeedCard state={state} />
      </DeliveryCard>
    );
  }

  const mission = state.mission;
  const status = deliveryStatus(state);
  const handlers: Record<NonNullable<DeliveryAction>, () => void> = {
    cancel: () => cancelDelivery(state),
    stop,
    start: () => driveAutonomous(state),
  };

  return (
    <DeliveryCard isBare={isBare} className={cn(isStale && 'opacity-50 grayscale')}>
      {stageSwitch}
      {mission.dock ? (
        <DockCard state={state} />
      ) : (
        <DeliveryHeader mission={mission} status={status} />
      )}

      {layout && <TrackMap state={state} layout={layout} />}

      {status.isRunning && (
        <>
          <div className="h-1 overflow-hidden rounded-full bg-line">
            <div
              className="h-full rounded-full bg-accent transition-[width] duration-500"
              style={{ width: `${Math.round(status.progress * 100)}%` }}
            />
          </div>
          {mission.next && mission.stage !== 'done' ? (
            <div className="flex items-center gap-2 text-base">
              <TurnIcon take={mission.next.take} />
              <span>
                <b className="font-semibold">
                  {mission.next.take === 'detour' ? 'Entre no desvio' : 'Siga reto'}
                </b>{' '}
                na saída {mission.next.exit}
              </span>
              <span className="ml-auto text-sm text-ink-3">para {mission.next.target}</span>
            </div>
          ) : (
            <p className="text-sm text-ink-2">{positionText(mission)}</p>
          )}
        </>
      )}

      {status.isPicking && (
        <div className="flex flex-col gap-2">
          <LetterPicker kind="pickup" state={state} />
          <LetterPicker kind="delivery" state={state} />
        </div>
      )}

      <CargoStatus state={state} />

      <Button
        size="lg"
        variant={status.action === 'start' ? 'accent' : 'soft'}
        disabled={!status.action}
        className="w-full disabled:bg-fill disabled:text-ink-3/70"
        onClick={status.action ? handlers[status.action] : undefined}
      >
        {status.actionLabel}
      </Button>
    </DeliveryCard>
  );
}

function DeliveryHeader({ mission, status }: DeliveryHeaderProps) {
  return (
    <>
      <div className="flex items-baseline justify-between gap-2.5">
        <h2
          className={cn(
            'text-xl font-semibold tracking-tight text-balance',
            !status.isSet && !status.isPicking && 'text-ink-2',
          )}
        >
          {status.title}
        </h2>
        {status.isRunning && (
          <span className="shrink-0 text-sm whitespace-nowrap text-ink-2 tabular-nums">
            {status.progressLabel}
          </span>
        )}
      </div>
      {status.note && (
        <p
          className={cn(
            '-mt-2 text-sm first-letter:uppercase',
            mission.hold ? 'text-stop-ink' : 'text-warn-ink',
          )}
        >
          {status.note}
        </p>
      )}
    </>
  );
}
