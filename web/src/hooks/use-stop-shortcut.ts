import { useEffect } from 'react';

import { stop } from '@/services/cart-commands';

export function useStopShortcut() {
  useEffect(() => {
    function handleKeyDown(event: KeyboardEvent) {
      if (event.code !== 'Space' || event.repeat) return;
      (document.activeElement as HTMLElement | null)?.blur();
      event.preventDefault();
      stop();
    }

    function handleKeyUp(event: KeyboardEvent) {
      if (event.code === 'Space') event.preventDefault();
    }

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);
}
