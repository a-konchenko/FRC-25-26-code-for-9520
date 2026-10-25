// Pure path edits (each returns a new PPPath; unknown fields are preserved).
import { pointAt } from './geometry';
import type { PPPath, PPWaypoint, XY } from './types';

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
  return {
    ...withWaypoints(p, wps),
    rotationTargets: p.rotationTargets.map((r) => ({ ...r, waypointRelativePos: shift(r.waypointRelativePos) })),
    eventMarkers: p.eventMarkers.map((m) => ({
      ...m,
      waypointRelativePos: shift(m.waypointRelativePos),
      endWaypointRelativePos: m.endWaypointRelativePos == null ? null : shift(m.endWaypointRelativePos),
    })),
  };
}

/** Delete waypoint i (a path keeps at least 2). Markers/targets past it shift back. */
export function deleteWaypoint(p: PPPath, i: number): PPPath {
  if (p.waypoints.length <= 2) return p;
  const wps = p.waypoints.filter((_, k) => k !== i);
  const maxPos = wps.length - 1;
  const shift = (pos: number) => Math.min(pos > i ? pos - 1 : pos, maxPos);
  return {
    ...withWaypoints(p, wps),
    rotationTargets: p.rotationTargets.map((r) => ({ ...r, waypointRelativePos: shift(r.waypointRelativePos) })),
    eventMarkers: p.eventMarkers.map((m) => ({
      ...m,
      waypointRelativePos: shift(m.waypointRelativePos),
      endWaypointRelativePos: m.endWaypointRelativePos == null ? null : shift(m.endWaypointRelativePos),
    })),
  };
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
