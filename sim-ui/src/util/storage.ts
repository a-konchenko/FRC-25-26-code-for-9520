// Per-browser conveniences (remembered settings). Storage can be unavailable
// (private windows, blocked site data), so every access is guarded and the
// app works without it.
const PREFIX = 'frc9520-sim:';

export function load<T>(key: string, fallback: T): T {
  try {
    const raw = window.localStorage.getItem(PREFIX + key);
    if (raw == null) return fallback;
    const v = JSON.parse(raw);
    return typeof fallback === 'object' && fallback !== null && !Array.isArray(fallback) ? { ...fallback, ...v } : v;
  } catch {
    return fallback;
  }
}

export function save(key: string, value: unknown): void {
  try {
    window.localStorage.setItem(PREFIX + key, JSON.stringify(value));
  } catch {
    /* ignore */
  }
}
