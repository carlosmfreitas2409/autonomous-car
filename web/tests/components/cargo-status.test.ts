import { describe, expect, test } from 'vitest';

import {
  cargoReadouts,
  manifestText,
  openHint,
} from '@/components/delivery-panel/utils/cargo-status';
import type { Cargo } from '@/types';

import { cartState } from '../fixtures/cart-state';

const cargo = (over: Partial<Cargo> = {}): Cargo => ({
  grams: 1420,
  stable: true,
  lid_closed: true,
  unlocked: false,
  fault: false,
  ...over,
});

describe('compartment', () => {
  test('reads nothing without a cargo node', () => {
    expect(cargoReadouts(null)).toEqual([]);
  });

  test('reads weight, lid and lock, flagging what is open', () => {
    expect(cargoReadouts(cargo())).toEqual([
      { text: '1.420 g', tone: 'normal' },
      { text: 'Tampa fechada', tone: 'normal' },
      { text: 'Travado', tone: 'normal' },
    ]);
    expect(cargoReadouts(cargo({ stable: false, lid_closed: false, unlocked: true }))).toEqual([
      { text: '~1.420 g', tone: 'muted' },
      { text: 'Tampa aberta', tone: 'warn' },
      { text: 'Destravado', tone: 'warn' },
    ]);
    expect(cargoReadouts(cargo({ fault: true }))[0]).toEqual({
      text: 'Balança sem resposta',
      tone: 'stop',
    });
  });

  test('names the load the truck left its pickup with', () => {
    const state = cartState();
    expect(manifestText(state.mission)).toBeNull();
    state.mission.manifest_g = 420;
    expect(manifestText(state.mission)).toBe('Carga 420 g · coletada em A');
  });

  test('opens by hand only stopped, with a compartment', () => {
    const state = cartState();
    expect(openHint(state)).toBe('Sem compartimento conectado');
    state.cargo = cargo();
    expect(openHint(state)).toBe('Só com o caminhão parado');
    state.drive.estopped = true;
    expect(openHint(state)).toBeNull();
  });
});
