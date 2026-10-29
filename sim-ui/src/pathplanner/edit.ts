// Pure path edits (each returns a new PPPath; unknown fields are preserved).
import { pointAt, rotationAt } from './geometry';
import type { PPConstraintZone, PPEventMarker, PPPath, PPWaypoint, XY } from './types';

const add = (a: XY, b: XY): XY => ({ x: a.x + b.x, y: a.y + b.y });
const sub = (a: XY, b: XY): XY => ({ x: a.x - b.x, y: a.y - b.y });
const len = (a: XY) => Math.hypot(a.x, a.y);
const scale = (a: XY, k: number): XY => ({ x: a.x * k, y: a.y * k });

function withWaypoints(p: PPPath, waypoints: PPWaypoint[]): PPPath {
  // first waypoint has no prevControl, last has no nextControl
  const n = waypoints.length;
  const fixed = waypoints.map((w, i) => ({
    ...w,
    prevControl: i === 0 ? null : w.prevControl ?? w.anchor,
    nextControl: i === n - 1 ? null : w.nextControl ?? w.anchor,
  }));
  return { ...p, waypoints: fixed };
}

/** Move a waypoint; its control handles move with it. */
export function moveAnchor(p: PPPath, i: number, to: XY): PPPath {
  const w = p.waypoints[i];
  const d = sub(to, w.anchor);
  const moved: PPWaypoint = {
    ...w,
    anchor: to,
    prevControl: w.prevControl ? add(w.prevControl, d) : null,
    nextControl: w.nextControl ? add(w.nextControl, d) : null,
  };
  return { ...p, waypoints: p.waypoints.map((x, k) => (k === i ? moved : x)) };
}

/**
 * Move one control handle. The opposite handle stays on the same line
 * (smooth curve) and keeps its length, unless `free` (Alt held).
 */
export function moveControl(p: PPPath, i: number, which: 'prev' | 'next', to: XY, free = false): PPPath {
  const w = p.waypoints[i];
  const other = which === 'prev' ? w.nextControl : w.prevControl;
  let otherNew = other;
  if (other && !free) {
    const dir = sub(w.anchor, to);
    const L = len(dir);
    if (L > 1e-6) otherNew = add(w.anchor, scale(dir, len(sub(other, w.anchor)) / L));
  }
  const moved: PPWaypoint = which === 'prev'
    ? { ...w, prevControl: to, nextControl: otherNew }
    : { ...w, nextControl: to, prevControl: otherNew };
  return { ...p, waypoints: p.waypoints.map((x, k) => (k === i ? moved : x)) };
}

/** Add a waypoint after the last one, with handles pointing along the new segment. */
export function appendWaypoint(p: PPPath, at: XY): PPPath {
  const last = p.waypoints[p.waypoints.length - 1];
  const d = scale(sub(at, last.anchor), 1 / 3);
  const lastNew: PPWaypoint = { ...last, nextControl: add(last.anchor, d) };
  const added: PPWaypoint = { anchor: at, prevControl: sub(at, d), nextControl: null, isLocked: false, linkedName: null };
  return withWaypoints(p, [...p.waypoints.slice(0, -1), lastNew, added]);
}

/** Insert a waypoint halfway along segment i (between waypoint i and i+1). */
export function insertWaypoint(p: PPPath, i: number): PPPath {
  if (i < 0 || i >= p.waypoints.length - 1) return p;
  const mid = pointAt(p, i + 0.5);
  const a = pointAt(p, i + 0.45), b = pointAt(p, i + 0.55);
  const tangent = sub(b, a);
  const t = len(tangent) > 1e-6 ? scale(tangent, 1 / len(tangent)) : { x: 1, y: 0 };
  const handle = len(sub(p.waypoints[i + 1].anchor, p.waypoints[i].anchor)) / 6;
  const added: PPWaypoint = {
    anchor: mid, prevControl: sub(mid, scale(t, handle)), nextControl: add(mid, scale(t, handle)),
    isLocked: false, linkedName: null,
  };
  const wps = [...p.waypoints.slice(0, i + 1), added, ...p.waypoints.slice(i + 1)];
  // positions after the new point shift by one; positions inside the split segment are rescaled
  const shift = (pos: number) => (pos <= i ? pos : pos >= i + 1 ? pos + 1 : i + (pos - i) * 2);
  return shiftPositions(withWaypoints(p, wps), p, shift);
}

/** Delete waypoint i (a path keeps at least 2). Markers/targets past it shift back. */
export function deleteWaypoint(p: PPPath, i: number): PPPath {
  if (p.waypoints.length <= 2) return p;
  const wps = p.waypoints.filter((_, k) => k !== i);
  const maxPos = wps.length - 1;
  const shift = (pos: number) => Math.min(pos > i ? pos - 1 : pos, maxPos);
  return shiftPositions(withWaypoints(p, wps), p, shift);
}

/**
 * Move everything placed by waypoint-relative position after a waypoint was
 * inserted or deleted: rotation targets, event markers, constraint zones and
 * point-towards zones (min/maxWaypointRelativePos). Unknown fields are kept.
 */
function shiftPositions(next: PPPath, p: PPPath, shift: (pos: number) => number): PPPath {
  const zone = (z: unknown) => {
    const r = z as Record<string, unknown>;
    const out: Record<string, unknown> = { ...r };
    for (const k of ['minWaypointRelativePos', 'maxWaypointRelativePos']) {
      if (typeof r[k] === 'number') out[k] = shift(r[k] as number);
    }
    return out;
  };
  return {
    ...next,
    rotationTargets: p.rotationTargets.map((r) => ({ ...r, waypointRelativePos: shift(r.waypointRelativePos) })),
    eventMarkers: p.eventMarkers.map((m) => ({
      ...m,
      waypointRelativePos: shift(m.waypointRelativePos),
      endWaypointRelativePos: m.endWaypointRelativePos == null ? null : shift(m.endWaypointRelativePos),
    })),
    constraintZones: (p.constraintZones ?? []).map(zone),
    pointTowardsZones: (p.pointTowardsZones ?? []).map(zone),
  };
}

// ── Stop & wait ─────────────────────────────────────────────────────────────
// PathPlannerLib can't pause in the middle of a path (event markers run
// alongside the driving). A "Stop & wait" marker is therefore a normal event
// marker with a wait command while editing; saving splits the path there into
// two paths that end/start at rest, joined by a linked waypoint, and the autos
// get a wait step between them.

export const STOP_MARKER_NAME = 'Stop & wait';

export function isStopMarker(m: PPEventMarker): boolean {
  return m.name === STOP_MARKER_NAME && m.command?.type === 'wait';
}

export function newStopMarker(pos: number, seconds: number): PPEventMarker {
  return { name: STOP_MARKER_NAME, waypointRelativePos: pos, endWaypointRelativePos: null, command: { type: 'wait', data: { waitTime: seconds } } };
}

/** Stop markers can't sit right on the start or end (that's just a wait step before/after the path). */
export const STOP_MARGIN = 0.05;

const lerp = (a: XY, b: XY, t: number): XY => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });

/**
 * Split a path at waypoint-relative position s (0 < s < last waypoint). The
 * curve is cut exactly (de Casteljau), so both parts trace the original path.
 * The first part ends at rest and the second starts at rest, both facing the
 * heading the path had at s. Rotation targets, event markers, constraint and
 * point-towards zones go to the part they fall in, with positions remapped.
 */
export function splitPath(p: PPPath, s: number, linkedName: string): [PPPath, PPPath] {
  const n = p.waypoints.length - 1;
  const wps = p.waypoints;
  const k = Math.round(s);
  const onWaypoint = Math.abs(s - k) < 0.01 && k > 0 && k < n;
  let a: PPWaypoint[], b: PPWaypoint[];
  let toA: (q: number) => number, toB: (q: number) => number;
  if (onWaypoint) {
    s = k;
    const w = wps[k];
    a = [...wps.slice(0, k), { ...w, nextControl: null, linkedName: w.linkedName ?? linkedName }];
    b = [{ ...w, prevControl: null, linkedName: w.linkedName ?? linkedName }, ...wps.slice(k + 1)];
    toA = (q) => q;
    toB = (q) => q - k;
  } else {
    const i = Math.min(Math.floor(s), n - 1), t = s - i;
    const P0 = wps[i].anchor, P1 = wps[i].nextControl ?? P0, P3 = wps[i + 1].anchor, P2 = wps[i + 1].prevControl ?? P3;
    const q1 = lerp(P0, P1, t), m12 = lerp(P1, P2, t), r2 = lerp(P2, P3, t);
    const q2 = lerp(q1, m12, t), r1 = lerp(m12, r2, t), mid = lerp(q2, r1, t);
    a = [...wps.slice(0, i), { ...wps[i], nextControl: q1 },
      { anchor: mid, prevControl: q2, nextControl: null, isLocked: false, linkedName }];
    b = [{ anchor: mid, prevControl: null, nextControl: r1, isLocked: false, linkedName },
      { ...wps[i + 1], prevControl: r2 }, ...wps.slice(i + 2)];
    toA = (q) => (q <= i ? q : i + (q - i) / t);
    toB = (q) => (q <= i + 1 ? (q - s) / (1 - t) : q - i);
  }
  const rot = normDeg((rotationAt(p, s) * 180) / Math.PI);
  const lenA = a.length - 1, lenB = b.length - 1;

  // A range [lo, hi] of the original clipped to one part (null = not in it)
  const clip = (lo: number, hi: number, first: boolean): [number, number] | null => {
    const [from, to] = first ? [Math.max(lo, 0), Math.min(hi, s)] : [Math.max(lo, s), Math.min(hi, n)];
    if (to - from < 1e-6) return null;
    const f = first ? toA : toB;
    return [f(from), f(to)];
  };
  const zones = (list: unknown[] | undefined, first: boolean) => (list ?? []).flatMap((z) => {
    const r = z as Record<string, unknown>;
    if (typeof r.minWaypointRelativePos !== 'number' || typeof r.maxWaypointRelativePos !== 'number') return first ? [r] : [];
    const c = clip(r.minWaypointRelativePos, r.maxWaypointRelativePos, first);
    return c ? [{ ...r, minWaypointRelativePos: c[0], maxWaypointRelativePos: c[1] }] : [];
  });
  const markers = (first: boolean): PPEventMarker[] => p.eventMarkers.flatMap((m) => {
    if (m.endWaypointRelativePos == null) {
      const inA = m.waypointRelativePos < s;
      return inA === first ? [{ ...m, waypointRelativePos: (first ? toA : toB)(m.waypointRelativePos) }] : [];
    }
    const c = clip(m.waypointRelativePos, m.endWaypointRelativePos, first);
    return c ? [{ ...m, waypointRelativePos: c[0], endWaypointRelativePos: c[1] }] : [];
  });
  const targets = (first: boolean) => p.rotationTargets
    .filter((r) => (first ? r.waypointRelativePos < s - 1e-6 : r.waypointRelativePos > s + 1e-6))
    .map((r) => ({ ...r, waypointRelativePos: (first ? toA : toB)(r.waypointRelativePos) }));
  const clampPos = (v: number, max: number) => Math.min(Math.max(v, 0), max);

  const partA: PPPath = {
    ...p, waypoints: a,
    rotationTargets: targets(true).map((r) => ({ ...r, waypointRelativePos: clampPos(r.waypointRelativePos, lenA) })),
    eventMarkers: markers(true), constraintZones: zones(p.constraintZones, true), pointTowardsZones: zones(p.pointTowardsZones, true),
    goalEndState: { ...p.goalEndState, velocity: 0, rotation: rot },
  };
  const partB: PPPath = {
    ...p, waypoints: b,
    rotationTargets: targets(false).map((r) => ({ ...r, waypointRelativePos: clampPos(r.waypointRelativePos, lenB) })),
    eventMarkers: markers(false), constraintZones: zones(p.constraintZones, false), pointTowardsZones: zones(p.pointTowardsZones, false),
    idealStartingState: { ...p.idealStartingState, velocity: 0, rotation: rot },
  };
  return [partA, partB];
}

/**
 * Split a path at all its Stop & wait markers. Returns the parts in driving
 * order and the wait (seconds) between each pair; the stop markers are gone.
 * `linkName(k)` names the linked waypoint joining part k and k+1.
 */
export function splitAtStops(p: PPPath, linkName: (k: number) => string): { parts: PPPath[]; waits: number[] } {
  const parts: PPPath[] = [], waits: number[] = [];
  let rest = p;
  for (;;) {
    // the earliest stop left; splitPath carries the later ones into the second part, remapped
    const stops = rest.eventMarkers.filter(isStopMarker);
    if (!stops.length) break;
    const m = stops.reduce((x, y) => (y.waypointRelativePos < x.waypointRelativePos ? y : x));
    const wait = Math.max(0, m.command?.data.waitTime ?? 0);
    const s = m.waypointRelativePos, n = rest.waypoints.length - 1;
    rest = { ...rest, eventMarkers: rest.eventMarkers.filter((x) => x !== m) };
    if (s <= STOP_MARGIN / 2) { // at the very start: a wait before this part
      if (parts.length) waits[waits.length - 1] += wait;
      continue;
    }
    if (s >= n - STOP_MARGIN / 2) continue; // at the very end: the path stops there anyway
    const [a, b] = splitPath(rest, s, linkName(parts.length));
    parts.push(a);
    waits.push(wait);
    rest = b;
  }
  parts.push(rest);
  return { parts, waits };
}

/** Linked waypoints (same linkedName) in `p`, by name → anchor. */
export function linkedAnchors(p: PPPath): Map<string, XY> {
  const out = new Map<string, XY>();
  for (const w of p.waypoints) if (w.linkedName) out.set(w.linkedName, w.anchor);
  return out;
}

/** Move this path's linked waypoints onto the given anchors (null = nothing to change). */
export function applyLinked(p: PPPath, anchors: Map<string, XY>): PPPath | null {
  let next = p, changed = false;
  p.waypoints.forEach((w, i) => {
    const to = w.linkedName ? anchors.get(w.linkedName) : undefined;
    if (to && Math.hypot(to.x - w.anchor.x, to.y - w.anchor.y) > 1e-6) { next = moveAnchor(next, i, to); changed = true; }
  });
  return changed ? next : null;
}

// ── Speed markers ───────────────────────────────────────────────────────────
// A speed marker changes the max speed (and acceleration) from its point on,
// until the next speed marker or the end of the path. It is stored as a
// PathPlanner constraint zone named "Speed marker …" whose end is kept at the
// next marker, so PathPlannerLib on the robot and the PathPlanner app read it
// as a normal zone. Other zones (made in the app) are left alone.

export const SPEED_ZONE_PREFIX = 'Speed marker';

export function isSpeedZone(z: unknown): z is PPConstraintZone {
  const r = z as PPConstraintZone;
  return typeof r?.name === 'string' && r.name.startsWith(SPEED_ZONE_PREFIX)
    && typeof r.minWaypointRelativePos === 'number' && typeof r.constraints === 'object';
}

export function speedZoneName(velocity: number): string {
  return `${SPEED_ZONE_PREFIX} (${+velocity.toFixed(2)} m/s)`;
}

export function newSpeedZone(p: PPPath, pos: number, velocity: number): PPConstraintZone {
  return {
    name: speedZoneName(velocity), minWaypointRelativePos: pos, maxWaypointRelativePos: p.waypoints.length - 1,
    constraints: { ...p.globalConstraints, maxVelocity: velocity, unlimited: false },
  };
}

/** Extend each speed zone to the next speed marker (or the path end); names follow the speed. */
export function normalizeSpeedZones(p: PPPath): PPPath {
  const zones = p.constraintZones ?? [];
  if (!zones.some(isSpeedZone)) return p;
  const n = p.waypoints.length - 1;
  const starts = zones.filter(isSpeedZone).map((z) => z.minWaypointRelativePos);
  let changed = false;
  const next = zones.map((z) => {
    if (!isSpeedZone(z)) return z;
    const min = Math.min(Math.max(z.minWaypointRelativePos, 0), n);
    const max = Math.min(n, ...starts.filter((s) => s > min + 1e-9));
    const name = speedZoneName(z.constraints.maxVelocity);
    if (min === z.minWaypointRelativePos && max === z.maxWaypointRelativePos && name === z.name) return z;
    changed = true;
    return { ...z, name, minWaypointRelativePos: min, maxWaypointRelativePos: max };
  });
  return changed ? { ...p, constraintZones: next } : p;
}

/** The max speed in effect at a position (first matching zone wins, like PathPlannerLib). */
export function maxSpeedAt(p: PPPath, pos: number): number {
  for (const z of p.constraintZones ?? []) {
    const r = z as PPConstraintZone;
    if (pos >= r.minWaypointRelativePos && pos <= r.maxWaypointRelativePos && r.constraints) return r.constraints.maxVelocity;
  }
  return p.globalConstraints.maxVelocity;
}

export function round(p: PPPath): PPPath {
  const r = (v: number) => Math.round(v * 10000) / 10000;
  const rp = (pt: XY | null) => (pt ? { ...pt, x: r(pt.x), y: r(pt.y) } : null);
  return { ...p, waypoints: p.waypoints.map((w) => ({ ...w, anchor: rp(w.anchor)!, prevControl: rp(w.prevControl), nextControl: rp(w.nextControl) })) };
}

export function normDeg(deg: number): number {
  let d = ((deg + 180) % 360 + 360) % 360 - 180;
  if (d === -180) d = 180;
  return Math.round(d * 10) / 10;
}
