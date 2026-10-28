import { type RefObject, useEffect, useState } from 'react';

import { VIDEO_SOCKET_PATH } from '@/constants';
import { socketUrl } from '@/utils/socket-url';

export type VideoSource = 'drive' | 'signs';

interface VideoStatus {
  isConnected: boolean;
  fps: number;
  lastFrameAt: number;
}

const MIN_BACKOFF_MS = 300;
const MAX_BACKOFF_MS = 3000;
const FPS_WINDOW_MS = 1000;

export function useVideo(source: VideoSource, canvas: RefObject<HTMLCanvasElement | null>) {
  const [status, setStatus] = useState<VideoStatus>({ isConnected: false, fps: 0, lastFrameAt: 0 });

  useEffect(() => {
    let isClosed = false;
    let socket: WebSocket | null = null;
    let backoff = MIN_BACKOFF_MS;
    let frames = 0;
    let windowStart = performance.now();
    let lastFrameAt = 0;

    function askForFrame(ws: WebSocket) {
      ws.send(JSON.stringify({ source, next: true }));
    }

    async function draw(data: Uint8Array<ArrayBuffer>) {
      const bitmap = await createImageBitmap(new Blob([data.subarray(1)], { type: 'image/jpeg' }));
      const target = canvas.current;
      if (target) {
        if (target.width !== bitmap.width || target.height !== bitmap.height) {
          target.width = bitmap.width;
          target.height = bitmap.height;
        }
        target.getContext('2d')?.drawImage(bitmap, 0, 0);
      }
      bitmap.close();
      frames += 1;
      lastFrameAt = performance.now();
    }

    function open() {
      const ws = new WebSocket(socketUrl(VIDEO_SOCKET_PATH));
      ws.binaryType = 'arraybuffer';
      socket = ws;

      ws.onopen = () => {
        backoff = MIN_BACKOFF_MS;
        setStatus((current) => ({ ...current, isConnected: true }));
        askForFrame(ws);
      };

      ws.onmessage = async (message) => {
        const data = new Uint8Array(message.data as ArrayBuffer);
        if (data.length > 1) await draw(data).catch(() => null);
        const now = performance.now();
        if (now - windowStart >= FPS_WINDOW_MS) {
          const fps = (frames * 1000) / (now - windowStart);
          frames = 0;
          windowStart = now;
          setStatus({ isConnected: true, fps, lastFrameAt });
        }
        if (ws.readyState === WebSocket.OPEN) askForFrame(ws);
      };

      ws.onclose = () => {
        socket = null;
        setStatus((current) => ({ ...current, isConnected: false, fps: 0 }));
        if (!isClosed) window.setTimeout(open, backoff);
        backoff = Math.min(backoff * 2, MAX_BACKOFF_MS);
      };
    }

    open();
    return () => {
      isClosed = true;
      socket?.close();
    };
  }, [source, canvas]);

  return status;
}
