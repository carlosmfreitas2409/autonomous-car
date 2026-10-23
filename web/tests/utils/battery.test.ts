import { describe, expect, test } from 'vitest';

import { readBattery } from '@/utils/battery';

describe('battery', () => {
  test('is judged healthy, low, empty, silent or not sent', () => {
    expect(readBattery({ percent: 78, volts: 12.43, age_s: 0.4 })).toEqual({
      percent: 78,
      volts: 12.43,
      tone: 'ok',
      note: null,
    });
    expect(readBattery({ percent: 25, volts: 12.0, age_s: 0.4 }).tone).toBe('warn');
    expect(readBattery({ percent: 12, volts: 11.9, age_s: 0.4 })).toMatchObject({
      tone: 'bad',
      note: 'Carregue logo',
    });
    expect(readBattery({ percent: 78, volts: 12.43, age_s: 9 })).toMatchObject({
      percent: 78,
      tone: 'warn',
      note: 'Sem leitura há 9 s',
    });
    expect(readBattery(null)).toEqual({
      percent: null,
      volts: null,
      tone: 'none',
      note: 'Sem leitura',
    });
  });
});
