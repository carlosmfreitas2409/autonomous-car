import { useEffect, useState } from 'react';

import { sendDrive } from '@/services/cart-commands';
import { fromGamepad } from '@/utils/gamepad';

const PERIOD_MS = 33;

export function useGamepadDrive(isActive: boolean) {
  const [gamepad, setGamepad] = useState<string | null>(null);

  useEffect(() => {
    const timer = window.setInterval(() => {
      const pad = [...(navigator.getGamepads?.() ?? [])].find((each) => each?.connected) ?? null;
      setGamepad(pad?.id ?? null);
      if (!isActive) return;

      const input = pad ? fromGamepad(pad) : { steering: 0, throttle: 0 };
      sendDrive(input.steering, input.throttle);
    }, PERIOD_MS);

    return () => window.clearInterval(timer);
  }, [isActive]);

  return gamepad;
}
