import { useEffect, useRef, useState } from 'react';

import { STRIP_COLOURS } from '@/constants';
import type { CartState } from '@/types';

import { Cart } from './cart';
import { ART_VIEWBOX, chaseCamera } from './utils/camera';
import { WorldScene } from './utils/world-scene';

interface WorldViewProps {
  state: CartState | null;
}

const MAX_DPR = 2;
const MAX_DT_S = 0.1;

export function WorldView({ state }: WorldViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const overlayRef = useRef<HTMLCanvasElement>(null);
  const stateRef = useRef(state);
  const [hasFailed, setHasFailed] = useState(false);
  const [size, setSize] = useState<{ width: number; height: number } | null>(null);

  useEffect(() => {
    stateRef.current = state;
  }, [state]);

  useEffect(() => {
    const canvas = canvasRef.current!;
    let scene: WorldScene;
    try {
      scene = new WorldScene(canvas, overlayRef.current!);
    } catch {
      setHasFailed(true);
      return;
    }

    const box = canvas.parentElement!;
    function fit() {
      const width = Math.max(1, box.clientWidth);
      const height = Math.max(1, box.clientHeight);
      scene.resize(width, height, Math.min(window.devicePixelRatio || 1, MAX_DPR));
      setSize((current) =>
        current?.width === width && current.height === height ? current : { width, height },
      );
    }
    fit();
    const observer = new ResizeObserver(fit);
    observer.observe(box);

    let frame = 0;
    let last = performance.now();
    function tick(now: number) {
      scene.frame(stateRef.current, Math.min((now - last) / 1000, MAX_DT_S), now);
      last = now;
      frame = requestAnimationFrame(tick);
    }
    frame = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      scene.dispose();
    };
  }, []);

  const lights = state?.lights;
  const camera = size && chaseCamera(size.width, size.height);
  const [vx, vy, vw] = ART_VIEWBOX;

  return (
    <div className="absolute inset-0">
      <canvas ref={canvasRef} className="absolute inset-0 size-full" />
      {camera && !hasFailed && (
        <Cart
          isBraking={!!lights?.brake}
          turnSignal={lights && lights.signal !== 'off' ? lights.signal : null}
          underglow={lights ? STRIP_COLOURS[lights.strip] : null}
          width={vw * camera.artScale}
          style={{
            left: camera.car[0] + vx * camera.artScale,
            top: camera.car[1] + vy * camera.artScale,
          }}
        />
      )}
      <canvas ref={overlayRef} className="pointer-events-none absolute inset-0 size-full" />
      {hasFailed && (
        <p className="absolute inset-0 grid place-items-center text-sm text-ink-3">
          The 3D view needs WebGL, which this browser has turned off.
        </p>
      )}
    </div>
  );
}
