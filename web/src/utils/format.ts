const formats = new Map<number, Intl.NumberFormat>();

export function formatNumber(value: number, decimals = 0) {
  let format = formats.get(decimals);
  if (!format) {
    format = new Intl.NumberFormat('pt-BR', {
      minimumFractionDigits: decimals,
      maximumFractionDigits: decimals,
    });
    formats.set(decimals, format);
  }
  return format.format(value);
}

export function formatClock(seconds: number) {
  const whole = Math.max(0, Math.floor(seconds));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, '0')}`;
}
