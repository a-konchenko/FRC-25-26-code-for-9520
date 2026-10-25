// Path geometry: cubic Bézier segments between waypoints, "waypoint-relative
// position" (0 = first waypoint, 1 = second, 1.5 = halfway along segment 2 —
// the unit PathPlanner uses for rotation targets and event markers), heading
// interpolation for the footprint preview, and rotational mirroring for red.
import type { PPPath, XY } from './types';

export interface PathSample { x: number; y: number; pos: number; rot: number; }

export function segmentCount(p: PPPath): number {
  return Math.max(0, p.waypoints.length - 1);
}

function controls(p: PPPath, i: number): [XY, XY, XY, XY] {
  const a = p.waypoints[i], b = p.waypoints[i + 1];
  return [a.anchor, a.nextControl ?? a.anchor, b.prevControl ?? b.anchor, b.anchor];
}

function bezier([p0, p1, p2, p3]: [XY, XY, XY, XY], t: number): XY {
  const u = 1 - t;
  const a = u * u * u, b = 3 * u * u * t, c = 3 * u * t * t, d = t * t * t;
  return { x: a * p0.x + b * p1.x + c * p2.x + d * p3.x, y: a * p0.y + b * p1.y + c * p2.y + d * p3.y };
}

/** Point at a waypoint-relative position. */
export function pointAt(p: PPPath, pos: number): XY {
  const n = segmentCount(p);
  if (n === 0) return p.waypoints[0]?.anchor ?? { x: 0, y: 0 };
  const clamped = Math.min(Math.max(pos, 0), n);
  const i = Math.min(Math.floor(clamped), n - 1);
  return bezier(controls(p, i), clamped - i);
}

/** Holonomic rotation (radians) along the path: start → rotation targets → end, shortest-way interpolation. */
export function rotationAt(p: PPPath, pos: number): number {
  const n = segmentCount(p);
  const keys = [
    { pos: 0, deg: p.idealStartingState?.rotation ?? 0 },
    ...[...p.rotationTargets].sort((a, b) => a.waypointRelativePos - b.waypointRelativePos)
      .map((t) => ({ pos: t.waypointRelativePos, deg: t.rotationDegrees })),
    { pos: n, deg: p.goalEndState?.rotation ?? 0 },
  ];
  for (let k = 0; k < keys.length - 1; k++) {
    const a = keys[k], b = keys[k + 1];
    if (pos <= b.pos || k === keys.length - 2) {
      const span = b.pos - a.pos;
      const f = span <= 1e-9 ? 1 : Math.min(Math.max((pos - a.pos) / span, 0), 1);
      let diff = ((b.deg - a.deg + 540) % 360) - 180;
      if (diff === -180) diff = 180;
      return ((a.deg + diff * f) * Math.PI) / 180;
    }
  }
  return 0;
}

/** Evenly spaced samples (by parameter) along the whole path. */
export function samplePath(p: PPPath, perSegment = 40): PathSample[] {
  const n = segmentCount(p);
  const out: PathSample[] = [];
  for (let i = 0; i < n; i++) {
    const c = controls(p, i);
    for (let k = 0; k < perSegment; k++) {
      const t = k / perSegment;
      const pt = bezier(c, t);
      out.push({ ...pt, pos: i + t, rot: rotationAt(p, i + t) });
    }
  }
  if (n > 0) {
    const end = p.waypoints[n].anchor;
    out.push({ ...end, pos: n, rot: rotationAt(p, n) });
  }
  return out;
}

export function pathLength(samples: PathSample[]): number {
  let L = 0;
  for (let i = 1; i < samples.length; i++) L += Math.hypot(samples[i].x - samples[i - 1].x, samples[i].y - samples[i - 1].y);
  return L;
}

/** Waypoint-relative position of the path point closest to (x, y). */
export function nearestPos(p: PPPath, x: number, y: number): { pos: number; dist: number } {
  const samples = samplePath(p, 60);
  let best = { pos: 0, dist: Infinity };
  for (const s of samples) {
    const d = Math.hypot(s.x - x, s.y - y);
    if (d < best.dist) best = { pos: s.pos, dist: d };
  }
  return best;
}

/** Rotational (180°) mirror about the field center — 2026 REBUILT symmetry. */
export function mirrorXY(pt: XY, L: number, W: number): XY {
  return { x: L - pt.x, y: W - pt.y };
}

export function mirrorRot(rad: number): number {
  return rad + Math.PI;
}

/** Sample poses + headings, mirrored for red when needed. */
export function samplesForAlliance(samples: PathSample[], red: boolean, L: number, W: number): PathSample[] {
  if (!red) return samples;
  return samples.map((s) => ({ ...mirrorXY(s, L, W), pos: s.pos, rot: mirrorRot(s.rot) }));
}
