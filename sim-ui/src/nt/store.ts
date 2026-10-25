// One shared NT4 connection for the whole app + React hooks.
import { useSyncExternalStore } from 'react';
import { NT4Client, type NTType, type NTValue } from './nt4';

// ?robot=<host> lets you point the UI at another machine (default: this one).
const params = new URLSearchParams(window.location.search);
export const ntHost = params.get('robot') ?? (window.location.hostname || 'localhost');

export const nt = new NT4Client(ntHost, 'sim-ui');
nt.subscribe(['/Sim/', '/SmartDashboard/', '/PathPlanner/'], true, 0.02);

// Coalesce NT updates (50 Hz+) into at most one React render per animation frame.
let frameVersion = 0;
let scheduled = false;
const reactListeners = new Set<() => void>();
nt.onChange(() => {
  if (scheduled) return;
  scheduled = true;
  requestAnimationFrame(() => {
    scheduled = false;
    frameVersion++;
    for (const l of reactListeners) l();
  });
});

function subscribe(l: () => void) {
  reactListeners.add(l);
  return () => reactListeners.delete(l);
}

/** Re-renders the component whenever any NT value changes (once per frame). */
export function useNTFrame(): number {
  return useSyncExternalStore(subscribe, () => frameVersion);
}

export function useNT<T extends NTValue>(topic: string): T | undefined;
export function useNT<T extends NTValue>(topic: string, fallback: T): T;
export function useNT<T extends NTValue>(topic: string, fallback?: T): T | undefined {
  useNTFrame();
  return (nt.get(topic) as T | undefined) ?? fallback;
}

export function useConnected(): boolean {
  useNTFrame();
  return nt.connected;
}

// ── Sending to the robot ────────────────────────────────────────────────────

/** A setting (latest value wins, re-sent after reconnect). */
export function setSetting(path: string, value: number | string | boolean, type?: NTType) {
  const t: NTType = type ?? (typeof value === 'number' ? 'double' : typeof value === 'string' ? 'string' : 'boolean');
  nt.set(`/Sim/Control/${path}`, t, value);
}

let seq = Date.now();
/** A one-shot action (never re-sent): /Sim/Control/<name> = [seq, ...args]. */
export function sendAction(name: string, args: number[] = []) {
  nt.set(`/Sim/Control/${name}`, 'double[]', [++seq, ...args], false);
}

export type Pose = [number, number, number];

export function asPose(v: NTValue | undefined): Pose | undefined {
  return Array.isArray(v) && v.length >= 3 ? [Number(v[0]), Number(v[1]), Number(v[2])] : undefined;
}
