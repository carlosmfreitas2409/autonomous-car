import { useState } from 'react';

import type { CartLink } from '@/hooks/use-cart-link';
import type { VideoSource } from '@/hooks/use-video';
import { cn } from '@/utils/cn';

import { AlertList } from './alert-list';
import { BatteryGauge } from './battery-gauge';
import { CameraCard } from './camera-card';
import { DeliveryPanel } from './delivery-panel';
import { DriveReadouts } from './drive-readouts';
import { DriveSwitch } from './drive-switch';
import { HealthPanel } from './health-panel';
import { ManualNote } from './manual-note';
import { PathControl } from './path-control';
import { SettingsButton } from './settings-dialog';
import { StatusPill } from './status-pill';
import { TrafficLightChip } from './traffic-light-chip';

interface DesktopLayoutProps {
  link: CartLink;
  gamepad: string | null;
}

const CAMERAS: VideoSource[] = ['drive', 'signs'];

export function DesktopLayout({ link, gamepad }: DesktopLayoutProps) {
  const [expanded, setExpanded] = useState<VideoSource | null>(null);
  const { state, live } = link;

  return (
    <>
      <div className="pointer-events-none absolute inset-y-6 right-6 z-10 flex flex-col items-end gap-3">
        <div className="flex items-stretch gap-3">
          <SettingsButton state={live} />
          <BatteryGauge state={state} isStale={link.isStale} />
          <DriveSwitch state={live} isConnected={link.isConnected} />
        </div>
        <HealthPanel link={link} />
        <DeliveryPanel state={state} layout={link.layout} isStale={link.isStale} />
      </div>

      <div className="pointer-events-none absolute inset-y-6 left-6 flex flex-col items-start gap-4">
        <div className="flex max-w-[calc(50vw-80px)] flex-none flex-col gap-4">
          <StatusPill
            state={live}
            isConnected={link.isConnected}
            isController={link.isController}
          />
          {state && (
            <div className={cn('flex flex-col gap-4', link.isStale && 'opacity-60')}>
              <DriveReadouts state={state} />
              <TrafficLightChip state={state} />
              <AlertList state={state} />
            </div>
          )}
        </div>

        <div className="mt-auto flex min-h-0 flex-col items-start gap-2.5">
          {CAMERAS.filter((source) => !expanded || source === expanded).map((source) => (
            <CameraCard
              key={source}
              state={state}
              source={source}
              isExpanded={expanded === source}
              onToggleExpanded={() => setExpanded(expanded === source ? null : source)}
            />
          ))}
        </div>
      </div>

      {live && (
        <div className="absolute bottom-6 left-1/2 -translate-x-1/2">
          {live.drive.mode === 'autonomous' && <PathControl state={live} />}
          {link.isManualHere && (
            <ManualNote state={live} gamepad={gamepad} isController={link.isController} />
          )}
        </div>
      )}
    </>
  );
}
