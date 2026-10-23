import type { CartState, Command, Letter, LightSettings, PathName, PidGains } from '@/types';

import { sendCommand } from './control-socket';

function asController(state: CartState, commands: Command[]) {
  if (commands.length === 0) return;
  if (state.control.controller !== state.you) sendCommand({ type: 'claim', force: true });
  for (const command of commands) sendCommand(command);
}

export function stop() {
  sendCommand({ type: 'estop', engaged: true });
}

export function resume(state: CartState) {
  if (state.drive.estopped) asController(state, [{ type: 'estop', engaged: false }]);
}

export function driveAutonomous(state: CartState) {
  const commands: Command[] = [];
  if (state.drive.mode !== 'autonomous') commands.push({ type: 'mode', mode: 'autonomous' });
  if (state.drive.estopped) commands.push({ type: 'estop', engaged: false });
  asController(state, commands);
}

export function driveManual(state: CartState) {
  const commands: Command[] = [];
  if (state.drive.mode !== 'manual' || state.drive.driver !== 'dashboard') {
    commands.push({ type: 'mode', mode: 'manual' });
  }
  if (state.drive.estopped) commands.push({ type: 'estop', engaged: false });
  asController(state, commands);
}

export function setPath(state: CartState, path: PathName) {
  asController(state, [{ type: 'path', path }]);
}

export function setLetters(state: CartState, pickup: Letter | null, delivery: Letter | null) {
  asController(state, [{ type: 'mission', pickup, delivery }]);
}

export function setLights(state: CartState, change: Partial<LightSettings>) {
  asController(state, [{ type: 'lights', ...change }]);
}

export function setSpeed(state: CartState, change: { base?: number; min?: number }) {
  asController(state, [{ type: 'speed', ...change }]);
}

export function setPid(state: CartState, change: Partial<PidGains>) {
  asController(state, [{ type: 'pid', ...change }]);
}

export function cancelDelivery(state: CartState) {
  if (!state.drive.estopped) stop();
  setLetters(state, null, null);
}

export function sendDrive(steering: number, throttle: number) {
  sendCommand({ type: 'drive', steering, throttle });
}

export function holdDeparture(state: CartState) {
  asController(state, [{ type: 'dock', action: 'wait' }]);
}

export function openCompartment(state: CartState) {
  asController(state, [{ type: 'open_cargo' }]);
}
