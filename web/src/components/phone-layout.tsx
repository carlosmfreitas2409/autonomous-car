import { useEffect, useState } from 'react';

import { CHALLENGE_NAMES } from '@/constants';
import type { CartLink } from '@/hooks/use-cart-link';
import { cn } from '@/utils/cn';
import { type Tone, checkHealth, summarizeHealth } from '@/utils/health';

import { AlertList } from './alert-list';
import { BatteryGauge } from './battery-gauge';
import { CameraCard } from './camera-card';
import { DeliveryPanel } from './delivery-panel';
import { DriveReadouts } from './drive-readouts';
import { DriveSwitch } from './drive-switch';
import { HealthPanel } from './health-panel';
import { PathControl } from './path-control';
import { SettingsButton } from './settings-dialog';
import { StatusPill } from './status-pill';
import { TrafficLightChip } from './traffic-light-chip';
import { Button } from './ui/button';
import { ButtonGroup, Panel } from './ui/panel';

interface PhoneLayoutProps {
  link: CartLink;
  isShort: boolean;
}

type Tab = 'delivery' | 'path' | 'health' | 'cameras';

const ATTENTION_DOT: Record<Tone, string | null> = {
  ok: null,
  none: null,
  warn: 'bg-warn',
  bad: 'bg-stop',
};

export function PhoneLayout({ link, isShort }: PhoneLayoutProps) {
  const [tab, setTab] = useState<Tab | null>(null);
  const { state, live } = link;
  const dock = state?.mission.dock;
  const dockKey = dock ? `${dock.letter}@${dock.since}` : null;

  // A truck that docks opens its card once for the person at the dock; they
  // may close it again.
  useEffect(() => {
    if (dockKey) setTab('delivery');
  }, [dockKey]);
  const health = summarizeHealth(checkHealth(link));
  const isAutonomous = state?.drive.mode !== 'manual';
  const challenge = state?.challenge ?? 'delivery';

  const tabs: { id: Tab; label: string; dot: string | null }[] = [
    { id: 'delivery', label: CHALLENGE_NAMES[challenge], dot: dock ? 'bg-accent' : null },
    ...(isAutonomous && challenge !== 'speed'
      ? [{ id: 'path' as const, label: 'Trajeto', dot: null }]
      : []),
    { id: 'health', label: 'Sistema', dot: ATTENTION_DOT[health.tone] },
    { id: 'cameras', label: 'Câmeras', dot: null },
  ];
  const open = tabs.some((item) => item.id === tab) ? tab : null;

  return (
    <>
      <div className="pointer-events-none absolute inset-x-4 top-[max(16px,env(safe-area-inset-top))] flex flex-col gap-4">
        <div className="flex items-stretch gap-2">
          <StatusPill
            state={live}
            isConnected={link.isConnected}
            className="min-w-0 flex-1 self-stretch"
          />
          <BatteryGauge state={state} isStale={link.isStale} />
          <SettingsButton state={live} className="h-auto" />
        </div>
        {state && (
          <div className={cn('flex flex-col items-start gap-4', link.isStale && 'opacity-60')}>
            <DriveReadouts state={state} isCompact />
            <TrafficLightChip state={state} />
            <AlertList state={state} max={isShort ? 1 : 3} />
          </div>
        )}
      </div>

      <div className="absolute inset-x-4 bottom-[max(16px,env(safe-area-inset-bottom))] flex flex-col gap-2">
        {open && (
          <Panel
            elevation="raised"
            className={cn(
              'overflow-auto rounded-3xl p-4',
              isShort ? 'max-h-[calc(100dvh-144px)] w-[min(420px,100%)] self-end' : 'max-h-[55dvh]',
            )}
          >
            {open === 'delivery' && (
              <DeliveryPanel state={state} layout={link.layout} isStale={link.isStale} isBare />
            )}
            {open === 'path' && live && <PathControl state={live} isFullWidth />}
            {open === 'health' && <HealthPanel link={link} isBare />}
            {open === 'cameras' && (
              <div className="flex flex-col gap-2">
                <CameraCard state={state} source="drive" isFullWidth />
                <CameraCard state={state} source="signs" isFullWidth />
              </div>
            )}
          </Panel>
        )}

        <div className={cn('flex gap-2', isShort ? 'flex-row' : 'flex-col')}>
          <ButtonGroup aria-label="Painéis" className="flex-1">
            {tabs.map((item) => {
              const isOpen = open === item.id;
              return (
                <Button
                  key={item.id}
                  aria-pressed={isOpen}
                  variant={isOpen ? 'selected' : 'ghost'}
                  className={cn('flex-1 px-0 text-sm', !isOpen && 'text-ink-2')}
                  onClick={() => setTab(isOpen ? null : item.id)}
                >
                  {item.label}
                  {item.dot && <span aria-hidden className={cn('size-2 rounded-full', item.dot)} />}
                </Button>
              );
            })}
          </ButtonGroup>
          <div className={cn(isShort && 'w-90 flex-none')}>
            <DriveSwitch state={live} isConnected={link.isConnected} isFullWidth />
          </div>
        </div>
      </div>
    </>
  );
}
