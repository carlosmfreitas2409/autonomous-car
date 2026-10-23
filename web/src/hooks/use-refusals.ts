import { useEffect, useState } from 'react';

import { refusalText } from '@/utils/loop-text';

import type { TimedReply } from './use-control';

interface Refusal {
  id: number;
  text: string;
}

const SHOW_MS = 4000;

export function useRefusals(replies: TimedReply[]) {
  const [refusals, setRefusals] = useState<Refusal[]>([]);
  const last = replies.at(-1);

  useEffect(() => {
    if (last?.type !== 'nack') return;
    const id = last.at;
    const text = refusalText(last);
    setRefusals((current) => [...current, { id, text }]);
    window.setTimeout(() => {
      setRefusals((current) => current.filter((refusal) => refusal.id !== id));
    }, SHOW_MS);
  }, [last]);

  return refusals;
}
