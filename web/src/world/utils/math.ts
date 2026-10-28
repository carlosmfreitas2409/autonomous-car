export function clamp(value: number, limit: number) {
  return Math.max(-limit, Math.min(limit, value));
}

export function clamp01(value: number) {
  return Math.max(0, Math.min(1, value));
}

export function smooth(t: number) {
  return t * t * (3 - 2 * t);
}

export function approach(current: number, target: number, dt: number, tau = 0.08) {
  const k = 1 - Math.exp(-dt / tau);
  return current + (target - current) * k;
}
