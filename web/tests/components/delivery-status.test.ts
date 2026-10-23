import { describe, expect, test } from 'vitest';

import { deliveryStatus } from '@/components/delivery-panel/utils/delivery-status';

import { cartState } from '../fixtures/cart-state';

describe('delivery status', () => {
  test('parked after a delivery, the letters stay live without a stop', () => {
    const state = cartState();
    state.mission.stage = 'done';
    state.mission.state = 'done';
    state.mission.parked_at = 'C';
    const status = deliveryStatus(state);
    expect(status.title).toBe('Entregue em C');
    expect(status.isPicking).toBe(true);
    expect(status.actionLabel).toBe('Limpar entrega');
  });

  test('picking needs a stop while driving a delivery', () => {
    const state = cartState();
    state.mission.step = 2;
    state.mission.route = ['exit 1: detour', 'A: pickup', 'C: delivery'];
    expect(deliveryStatus(state)).toMatchObject({
      title: 'Indo coletar em A',
      isPicking: false,
      progressLabel: 'Etapa 3 de 3',
      actionLabel: 'Cancelar entrega',
    });
  });

  test('docked, even e-stopped at the start of an order, the letters are put away', () => {
    const state = cartState();
    state.drive.estopped = true;
    state.mission.dock = {
      letter: 'C',
      role: null,
      since: 0,
      released: false,
      lid_opened: false,
      check_failed: null,
      override: false,
    };
    expect(deliveryStatus(state)).toMatchObject({ isPicking: false, action: 'cancel' });
  });
});
