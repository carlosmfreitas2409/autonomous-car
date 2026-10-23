import { useEffect, useState } from 'react';

import { CONTRACT_VERSION, STALE_MS } from '@/constants';

import { useControl } from './use-control';

const REFRESH_MS = 250;

export type CartLink = ReturnType<typeof useCartLink>;

export function useCartLink() {
  const { isConnected, hello, state, receivedAt, rttMs, replies } = useControl();
  const [now, setNow] = useState(() => performance.now());

  useEffect(() => {
    const timer = window.setInterval(() => setNow(performance.now()), REFRESH_MS);
    return () => window.clearInterval(timer);
  }, []);

  const ageS = receivedAt ? (now - receivedAt) / 1000 : Infinity;
  const isStale = !isConnected || ageS * 1000 > STALE_MS;
  const live = isStale ? null : state;

  return {
    state,
    live,
    layout: hello?.layout ?? null,
    mismatchVersion: hello && hello.version !== CONTRACT_VERSION ? hello.version : null,
    isConnected,
    isStale,
    ageS,
    rttMs,
    replies,
  };
}
