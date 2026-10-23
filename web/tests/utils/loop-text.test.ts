import { describe, expect, test } from 'vitest';

import { missionLine, refusalText, stopTitle } from '@/utils/loop-text';

import { cartState } from '../fixtures/cart-state';

describe('loop text', () => {
  test('words the stop by where it comes from', () => {
    const state = cartState();
    expect(stopTitle(state)).toBeNull();
    state.drive.stop_text = 'lane lost';
    state.drive.stop_source = 'controller';
    expect(stopTitle(state)).toBe('Faixa perdida');
    state.drive.stop_text = 'stopped at sign';
    state.drive.stop_source = 'sign';
    state.policy = { ...state.policy, state: 'holding', reason: 'stopped at sign', hold: true };
    expect(stopTitle(state)).toBe('Parado na placa de pare');
    state.drive.stop_text = 'holding 1.2s';
    state.drive.stop_source = 'holding';
    state.path.holding_s = 1.24;
    expect(stopTitle(state)).toBe('Segurando 1,2 s');
  });

  test('words the mission from its state, not its English reason', () => {
    const state = cartState();
    expect(missionLine(state.mission)).toBe('Indo coletar em A');
    state.mission.state = 'docked';
    state.mission.dock = {
      letter: 'A',
      role: 'pickup',
      since: 0,
      released: false,
      lid_opened: false,
      check_failed: null,
      override: false,
    };
    expect(missionLine(state.mission)).toBe('Carregando em A');
    state.mission.state = 'checking';
    state.mission.check = 'blocked';
    expect(missionLine(state.mission)).toBe('Frente bloqueada · afaste-se');
  });

  test('words refusals, and keeps what it has not been taught', () => {
    expect(refusalText({ cmd: 'dock', reason: 'e-stopped, resume first' })).toBe(
      'Doca · Caminhão parado, retome primeiro',
    );
    expect(refusalText({ cmd: 'mystery', reason: 'something new' })).toBe(
      'mystery · something new',
    );
  });
});
