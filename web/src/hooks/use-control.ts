import { create } from 'zustand';

import type { CartState, Hello, Reply } from '@/types';

export interface TimedReply extends Reply {
  at: number;
}

interface ControlState {
  isConnected: boolean;
  hello: Hello | null;
  state: CartState | null;
  receivedAt: number;
  rttMs: number | null;
  replies: TimedReply[];
}

export const useControl = create<ControlState>()(() => ({
  isConnected: false,
  hello: null,
  state: null,
  receivedAt: 0,
  rttMs: null,
  replies: [],
}));
