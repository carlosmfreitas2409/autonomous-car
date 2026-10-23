import type { CartState, Command, Letter, LightSettings, PathName, PidGains } from '@/types';

import { sendCommand } from './control-socket';

export function stop() {
  sendCommand({ type: 'estop', engaged: true });
}

export function resume() {
  sendCommand({ type: 'estop', engaged: false });
}

function driveIn(state: CartState, mode: Command & { type: 'mode' }) {
  if (state.drive.mode !== mode.mode) sendCommand(mode);
  if (state.drive.estopped) resume();
}

export function driveAutonomous(state: CartState) {
  driveIn(state, { type: 'mode', mode: 'autonomous' });
}

export function driveManual(state: CartState) {
  driveIn(state, { type: 'mode', mode: 'manual' });
}

export function setPath(path: PathName) {
  sendCommand({ type: 'path', path });
}

export function setLetters(pickup: Letter | null, delivery: Letter | null) {
  sendCommand({ type: 'mission', pickup, delivery });
}

export function setLights(change: Partial<LightSettings>) {
  sendCommand({ type: 'lights', ...change });
}

export function setSpeed(change: { base?: number; min?: number }) {
  sendCommand({ type: 'speed', ...change });
}

export function setPid(change: Partial<PidGains>) {
  sendCommand({ type: 'pid', ...change });
}

export function cancelDelivery(state: CartState) {
  if (!state.drive.estopped) stop();
  setLetters(null, null);
}

export function holdDeparture() {
  sendCommand({ type: 'dock', action: 'wait' });
}

export function openCompartment() {
  sendCommand({ type: 'open_cargo' });
}
