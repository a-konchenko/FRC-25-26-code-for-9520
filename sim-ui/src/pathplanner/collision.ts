// Robot footprint vs field elements (Plan tab warning: "this path drives
// through the HUB"). Separating-axis test between the rotated bumper
// rectangle and each convex element polygon.
import type { FieldElement } from '../field/fieldData';
import type { PathSample } from './geometry';

type Pt = [number, number];

export function robotCorners(x: number, y: number, rot: number, L: number, W: number): Pt[] {
  const c = Math.cos(rot), s = Math.sin(rot);
  return ([[L / 2, W / 2], [L / 2, -W / 2], [-L / 2, -W / 2], [-L / 2, W / 2]] as Pt[])
    .map(([dx, dy]) => [x + dx * c - dy * s, y + dx * s + dy * c] as Pt);
}

function axes(poly: Pt[]): Pt[] {
  return poly.map((p, i) => {
    const q = poly[(i + 1) % poly.length];
    return [-(q[1] - p[1]), q[0] - p[0]] as Pt;
  });
}

function overlaps(a: Pt[], b: Pt[]): boolean {
  for (const [ax, ay] of [...axes(a), ...axes(b)]) {
    let aMin = Infinity, aMax = -Infinity, bMin = Infinity, bMax = -Infinity;
    for (const [x, y] of a) { const d = x * ax + y * ay; aMin = Math.min(aMin, d); aMax = Math.max(aMax, d); }
    for (const [x, y] of b) { const d = x * ax + y * ay; bMin = Math.min(bMin, d); bMax = Math.max(bMax, d); }
    if (aMax < bMin || bMax < aMin) return false;
  }
  return true;
}

export interface Collision { pos: number; x: number; y: number; what: string; }

/**
 * Samples where the bumpers overlap a solid element or leave the field.
 * BUMPs are drivable and TRENCHes are low-clearance (robot ≤ 22.25 in fits),
 * so only `solid` elements count.
 */
export function findCollisions(samples: PathSample[], L: number, W: number, elements: FieldElement[], fieldL: number, fieldW: number): Collision[] {
  const solids = elements.filter((e) => e.type === 'solid').map((e) => ({ name: `${e.alliance} ${e.name}`, poly: e.poly as Pt[] }));
  const out: Collision[] = [];
  let lastX = Infinity, lastY = Infinity;
  for (const s of samples) {
    if (Math.hypot(s.x - lastX, s.y - lastY) < 0.05) continue; // ~5 cm spacing
    lastX = s.x; lastY = s.y;
    const corners = robotCorners(s.x, s.y, s.rot, L, W);
    const outside = corners.some(([x, y]) => x < 0 || y < 0 || x > fieldL || y > fieldW);
    if (outside) { out.push({ pos: s.pos, x: s.x, y: s.y, what: 'field wall' }); continue; }
    for (const el of solids) {
      if (overlaps(corners, el.poly)) { out.push({ pos: s.pos, x: s.x, y: s.y, what: el.name }); break; }
    }
  }
  return out;
}
