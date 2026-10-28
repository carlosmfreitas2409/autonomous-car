import type { Cargo, CartState, Mission } from '@/types';
import { formatNumber } from '@/utils/format';

export type CargoTone = 'normal' | 'muted' | 'warn' | 'stop';

export interface CargoReadout {
  text: string;
  tone: CargoTone;
}

export function cargoReadouts(cargo: Cargo | null): CargoReadout[] {
  if (!cargo) return [];
  const grams = `${formatNumber(cargo.grams)} g`;
  return [
    cargo.fault
      ? { text: 'Balança sem resposta', tone: 'stop' }
      : { text: cargo.stable ? grams : `~${grams}`, tone: cargo.stable ? 'normal' : 'muted' },
    cargo.lid_closed
      ? { text: 'Tampa fechada', tone: 'normal' }
      : { text: 'Tampa aberta', tone: 'warn' },
    cargo.unlocked ? { text: 'Destravado', tone: 'warn' } : { text: 'Travado', tone: 'normal' },
  ];
}

export function manifestText(mission: Mission) {
  if (mission.manifest_g === null) return null;
  const from = mission.pickup ? ` · coletada em ${mission.pickup}` : '';
  return `Carga ${formatNumber(mission.manifest_g)} g${from}`;
}

export function openHint(state: CartState) {
  if (!state.cargo) return 'Sem compartimento conectado';
  if (!state.drive.estopped) return 'Só com o caminhão parado';
  return null;
}
