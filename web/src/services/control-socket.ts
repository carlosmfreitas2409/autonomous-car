import { CONTROL_SOCKET_PATH } from '@/constants';
import type { Command, ServerMessage } from '@/types';
import { useControl } from '@/hooks/use-control';
import { socketUrl } from '@/utils/socket-url';

const PING_MS = 500;
const MIN_BACKOFF_MS = 300;
const MAX_BACKOFF_MS = 3000;
const MAX_REPLIES = 10;

let socket: WebSocket | null = null;
let backoff = MIN_BACKOFF_MS;
let nextId = 1;
let pingTimer: number | undefined;

export function connectControl() {
  const ws = new WebSocket(socketUrl(CONTROL_SOCKET_PATH));
  socket = ws;

  ws.onopen = () => {
    backoff = MIN_BACKOFF_MS;
    useControl.setState({ isConnected: true });
    window.clearInterval(pingTimer);
    pingTimer = window.setInterval(
      () => sendCommand({ type: 'ping', t: performance.now() }),
      PING_MS,
    );
  };

  ws.onmessage = (message) => receive(JSON.parse(message.data));

  ws.onclose = () => {
    window.clearInterval(pingTimer);
    socket = null;
    useControl.setState({ isConnected: false, rttMs: null });
    window.setTimeout(connectControl, backoff);
    backoff = Math.min(backoff * 2, MAX_BACKOFF_MS);
  };
}

export function sendCommand(command: Command) {
  if (socket?.readyState !== WebSocket.OPEN) return false;
  socket.send(JSON.stringify({ ...command, id: nextId++ }));
  return true;
}

function receive(message: ServerMessage) {
  switch (message.type) {
    case 'state':
      useControl.setState({ state: message, receivedAt: performance.now() });
      break;
    case 'hello':
      useControl.setState({ hello: message });
      break;
    case 'pong':
      useControl.setState({ rttMs: performance.now() - message.t });
      break;
    case 'ack':
    case 'nack':
      if (message.cmd === 'ping') break;
      useControl.setState((current) => ({
        replies: [...current.replies, { ...message, at: performance.now() }].slice(-MAX_REPLIES),
      }));
      break;
  }
}
