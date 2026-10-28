import type { CartState } from '@/types';

import { WorldView } from '@/world/world-view';

interface WorldBackdropProps {
  state: CartState | null;
  isStale: boolean;
}

const STOP_TINT =
  'radial-gradient(ellipse at center, rgb(229 72 77 / 0) 55%, rgb(229 72 77 / 0.16) 100%)';

export function WorldBackdrop({ state, isStale }: WorldBackdropProps) {
  const isStopped = !!state?.drive.estopped && !isStale;

  return (
    <>
      <section aria-label="Vista do mundo" className="absolute inset-0 bg-world">
        <WorldView state={state} />
      </section>
      <div
        aria-hidden
        className="pointer-events-none absolute inset-0 transition-[background] duration-300"
        style={{ background: isStopped ? STOP_TINT : 'transparent' }}
      />
      {isStale && <div className="pointer-events-none absolute inset-0 bg-page/55" />}
    </>
  );
}
