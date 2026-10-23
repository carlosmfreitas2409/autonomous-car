import { DesktopLayout } from '@/components/desktop-layout';
import { LinkBanner } from '@/components/link-banner';
import { PhoneLayout } from '@/components/phone-layout';
import { RefusalToasts } from '@/components/refusal-toasts';
import { WorldBackdrop } from '@/components/world-backdrop';
import { useCartLink } from '@/hooks/use-cart-link';
import { useGamepadDrive } from '@/hooks/use-gamepad-drive';
import { useMediaQuery } from '@/hooks/use-media-query';
import { useStopShortcut } from '@/hooks/use-stop-shortcut';
import { cn } from '@/utils/cn';

const COMPACT_QUERY = '(max-width: 767px), (max-height: 500px)';
const SHORT_QUERY = '(max-height: 500px)';

export function App() {
  const link = useCartLink();
  const isCompact = useMediaQuery(COMPACT_QUERY);
  const isShort = useMediaQuery(SHORT_QUERY);
  const gamepad = useGamepadDrive(link.isDrivingHere);
  useStopShortcut();

  return (
    <div
      className={cn(
        'relative h-dvh overflow-hidden tabular-nums',
        !isCompact && 'min-h-120 min-w-215',
      )}
    >
      <WorldBackdrop state={link.state} isStale={link.isStale} />

      {isCompact ? (
        <PhoneLayout link={link} gamepad={gamepad} isShort={isShort} />
      ) : (
        <DesktopLayout link={link} gamepad={gamepad} />
      )}

      {(link.isStale || link.mismatchVersion !== null) && (
        <LinkBanner link={link} isCompact={isCompact} />
      )}

      <RefusalToasts replies={link.replies} isCompact={isCompact} />
    </div>
  );
}
