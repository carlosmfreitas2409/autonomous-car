export function readPalette() {
  const css = getComputedStyle(document.documentElement);
  const token = (name: string, fallback: string) => css.getPropertyValue(name).trim() || fallback;
  return {
    sans: token('--font-sans', 'sans-serif'),
    world: token('--color-world', '#e6e3dc'),
    road: token('--color-road', '#fcfbf9'),
    marking: token('--color-marking', '#8e897f'),
    accent: token('--color-accent', '#0fa898'),
    ink: token('--color-ink', '#16171a'),
    ink2: token('--color-ink-2', '#5d6068'),
    panel: token('--color-panel', '#ffffff'),
    line: token('--color-line-strong', '#cdd0d6'),
    stop: token('--color-stop', '#e5484d'),
    stopInk: token('--color-stop-ink', '#c8252b'),
    stopSoft: token('--color-stop-soft', '#fde4e4'),
    warn: token('--color-warn', '#f0a020'),
    warnInk: token('--color-warn-ink', '#8c5300'),
    ok: token('--color-ok', '#2fbf71'),
  };
}

export type Palette = ReturnType<typeof readPalette>;
