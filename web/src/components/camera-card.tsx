import { useRef } from 'react';

import { type VideoSource, useVideo } from '@/hooks/use-video';
import type { CartState } from '@/types';
import { isSpeedTest } from '@/utils/challenge';
import { cn } from '@/utils/cn';

import { Button } from './ui/button';

interface CameraCardProps {
  state: CartState | null;
  source: VideoSource;
  isExpanded?: boolean;
  isFullWidth?: boolean;
  onToggleExpanded?: () => void;
}

const TITLES: Record<VideoSource, string> = { drive: 'Câmera da pista', signs: 'Câmera de placas' };
const EXPAND_ICON = 'M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7';
const SHRINK_ICON = 'M4 14h6v6M20 10h-6V4M14 10l7-7M3 21l7-7';
const NO_SIGNAL_S = 1.5;

function overlayMessage(
  state: CartState | null,
  source: VideoSource,
  isConnected: boolean,
  frameAgeS: number,
) {
  if (!isConnected) return 'Sem conexão de vídeo';
  if (source === 'drive' && state?.drive.estopped) return 'Pausada com o caminhão parado';
  if (source === 'signs' && state && isSpeedTest(state)) return 'Pausada na prova de velocidade';
  if (source === 'signs' && state && !state.signs.camera_open) {
    return state.signs.status
      ? `Câmera de placas: ${state.signs.status}`
      : 'Câmera de placas desligada';
  }
  if (frameAgeS > NO_SIGNAL_S) return 'Sem sinal';
  return null;
}

export function CameraCard({
  state,
  source,
  isExpanded = false,
  isFullWidth = false,
  onToggleExpanded,
}: CameraCardProps) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const video = useVideo(source, canvas);
  const frameAgeS = video.lastFrameAt ? (performance.now() - video.lastFrameAt) / 1000 : Infinity;
  const message = overlayMessage(state, source, video.isConnected, frameAgeS);

  return (
    <section
      aria-label={TITLES[source]}
      className={cn(
        '@container pointer-events-auto relative aspect-video min-h-0 flex-initial overflow-hidden rounded-2xl bg-well shadow-card',
        isFullWidth && 'w-full',
        !isFullWidth && (isExpanded ? 'h-[min(342px,25.875vw)]' : 'h-[min(171px,14.625vw)]'),
      )}
    >
      <canvas ref={canvas} className="absolute inset-0 size-full object-contain" />

      {message && (
        <div className="absolute inset-0 grid place-items-center bg-well/85 p-4 text-center text-sm font-medium text-ink-2">
          {message}
        </div>
      )}

      <div className="pointer-events-none absolute top-2 left-2 flex items-center gap-2 rounded-lg bg-black/60 px-2 py-1 text-xs font-medium whitespace-nowrap text-white">
        <span
          aria-hidden
          className={cn('size-2 rounded-full', message ? 'bg-white/50' : 'bg-stop')}
        />
        {TITLES[source]}
        <span className="hidden @[208px]:inline">· {video.fps.toFixed(0)} fps</span>
      </div>

      {onToggleExpanded && (
        <Button
          variant="overlay"
          size="icon"
          aria-label={isExpanded ? 'Reduzir' : 'Ampliar'}
          className="absolute top-2 right-2"
          onClick={onToggleExpanded}
        >
          <svg
            aria-hidden
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth={2.2}
            strokeLinecap="round"
          >
            <path d={isExpanded ? SHRINK_ICON : EXPAND_ICON} />
          </svg>
        </Button>
      )}
    </section>
  );
}
