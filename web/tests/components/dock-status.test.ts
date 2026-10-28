import { describe, expect, test } from 'vitest';

import { dockStatus } from '@/components/delivery-panel/utils/dock-status';
import type { Cargo, CartState, Dock } from '@/types';

import { cartState } from '../fixtures/cart-state';

const closed: Cargo = { grams: 0, stable: true, lid_closed: true, unlocked: true, fault: false };

function docked(dock: Partial<Dock> = {}, cargo: Cargo | null = null): CartState {
  const state = cartState();
  state.t = 183;
  state.mission.state = 'docked';
  state.mission.dock = {
    letter: 'A',
    role: 'pickup',
    since: 100,
    released: false,
    lid_opened: false,
    check_failed: null,
    override: false,
    ...dock,
  };
  state.cargo = cargo;
  return state;
}

describe('dock card', () => {
  test('is absent away from a dock', () => {
    expect(dockStatus(cartState())).toBeNull();
  });

  test('loading without a compartment waits for a release given elsewhere', () => {
    expect(dockStatus(docked())).toEqual({
      title: 'Carregando em A',
      waited: 'Esperando 1:23',
      line: 'Carregue e aguarde a liberação',
      tone: 'normal',
      hint: null,
      canWait: false,
    });
  });

  test('with a compartment, closing the lid is the release', () => {
    expect(dockStatus(docked({ letter: 'C', role: 'delivery' }, closed))).toMatchObject({
      title: 'Descarregando em C',
      line: 'Abra, retire a carga e feche a tampa',
      hint: 'Fechar a tampa libera o caminhão',
      canWait: false,
    });
    expect(dockStatus(docked({}, closed))?.line).toBe('Abra, carregue e feche a tampa');
    expect(dockStatus(docked({ lid_opened: true }, { ...closed, lid_closed: false }))?.line).toBe(
      'Feche a tampa quando terminar',
    );
  });

  test('a failed weight check says why and asks for the lid again', () => {
    expect(dockStatus(docked({ check_failed: 'empty' }, closed))).toMatchObject({
      line: 'Nenhuma carga detectada',
      tone: 'stop',
      hint: 'Carregue e feche a tampa de novo',
    });
    const loaded = dockStatus(docked({ role: 'delivery', check_failed: 'still_loaded' }, closed));
    expect(loaded?.line).toBe('Ainda há carga no compartimento');
    const overridden = dockStatus(docked({ check_failed: 'empty', override: true }, closed));
    expect(overridden?.line).toBe('Abra, carregue e feche a tampa');
  });

  test('a new order leaving the parked dock waits for its release', () => {
    const state = docked({ letter: 'C', role: null });
    state.mission.target = 'B';
    expect(dockStatus(state)).toMatchObject({
      title: 'Saindo de C',
      line: 'Aguardando liberação para seguir até B',
    });
  });

  test('checking names what holds the truck and offers to wait', () => {
    const state = docked({ released: true }, closed);
    state.mission.state = 'checking';
    const lines = (['lid', 'weighing', 'announcing', 'clearing', 'blocked'] as const).map(
      (check) => {
        state.mission.check = check;
        return [dockStatus(state)?.line, dockStatus(state)?.tone];
      },
    );
    expect(lines).toEqual([
      ['Aguardando fechar a tampa', 'warn'],
      ['Pesando a carga', 'normal'],
      ['Avisando que vai sair', 'normal'],
      ['Conferindo se a frente está livre', 'normal'],
      ['Frente bloqueada · afaste-se', 'stop'],
    ]);
    expect(dockStatus(state)).toMatchObject({ title: 'Saindo de A', waited: null, canWait: true });
  });

  test('e-stopped, it says so', () => {
    const state = docked({}, closed);
    state.drive.estopped = true;
    state.mission.state = 'idle';
    expect(dockStatus(state)).toMatchObject({
      title: 'Carregando em A',
      hint: 'Parado · retome para seguir',
      canWait: false,
    });
  });
});
