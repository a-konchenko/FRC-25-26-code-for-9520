// Generates field-2026.json (2026 REBUILT) for the web UI and the Java sim.
//
//   node scripts/gen-field.mjs
//
// Sources
//   [TAGS]   WPILib 2026.2.1 apriltag-java: 2026-rebuilt-andymark.json /
//            2026-rebuilt-welded.json (the same layout the robot code loads)
//   [GM]     2026 FRC Game Manual, section 5 ARENA (TU22)
//            https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
//
// Coordinates: WPILib blue origin, meters. +X from the blue alliance wall
// toward red, +Y to the left as seen from blue. The field is rotationally
// symmetric (checked against the tag layout: tag 31 ↔ tag 15, etc.), so red
// elements are blue elements rotated 180° about the field center.
//
// Every element lists where its numbers come from. Elements marked
// approx:true have a position the manual text does not pin down — check them
// against the official Field Dimension Drawings and fix here.

import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const IN = 0.0254;
const here = dirname(fileURLToPath(import.meta.url));

// [TAGS] "field" block of each layout file, plus the tag coordinates used as
// anchors (hub faces from tags 25/19 (x) and 18/21 (y); tower from 31/32;
// outpost from 29/30). The two variants differ by ~1.5 cm.
const VARIANTS = {
  andymark: {
    length: 16.518, width: 8.043,
    hubX: [4.008, 5.215], hubY: [3.418, 4.625], towerY: [3.730, 4.162], outpostY: [0.651, 1.083],
  },
  welded: {
    length: 16.541, width: 8.069,
    hubX: [4.022, 5.229], hubY: [3.431, 4.638], towerY: [3.746, 4.178], outpostY: [0.666, 1.098],
  },
};
const mid = ([a, b]) => (a + b) / 2;

const rect = (x0, y0, x1, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];

function blueElements(v) {
  const W = v.width;
  const cy = mid(v.hubY);

  // HUB — [GM] 47in x 47in. Center from the 8 hub tags (IDs 18-21, 24-27):
  // faces at x = 4.008 / 5.215, y = 3.418 / 4.625 → center (4.6115, W/2).
  const hubX = mid(v.hubX);
  const hubHalf = (47 * IN) / 2;

  // BUMP — [GM] 73.0in wide (along Y) x 44.4in deep (along X) x 6.513in,
  // "on either side of the HUB", centered on the hub's X.
  const bumpW = 73.0 * IN, bumpD = 44.4 * IN;

  // TRENCH — [GM] 65.65in wide x 47.0in deep x 40.25in tall, extends from
  // the guardrail to the BUMP; opening under each arm 50.34in x 22.25in.
  // Trench tags 17/22/23/28 sit at the hub's X. Clipped where it meets the bump.
  const trenchD = 47.0 * IN;

  // TOWER — [GM] 49.25in wide x 45.0in deep x 78.25in; base 39.0in x 45.18in.
  // Integrated in the alliance wall between DS2 and DS3. Center Y from
  // tower tags 31/32 (y = 3.730 / 4.162).
  const towerY = mid(v.towerY);

  // OUTPOST — [GM] at the end of the field where the guardrail meets the
  // alliance wall. Y from outpost tags 29/30 (y = 0.651 / 1.083).
  // Corral interior 35.8in wide x 37.6in deep.
  const outpostY = mid(v.outpostY);

  // DEPOT — [GM] 42.0in wide x 27.0in deep, "along the ALLIANCE WALL".
  // The manual text gives no position; placed on the far side of the TOWER
  // from the OUTPOST. approx — verify against GM Figure 5-11 / drawings.
  const depotCY = 5.96;

  const bumpY0 = cy + hubHalf;
  const els = [
    {
      id: 'hub', name: 'HUB', type: 'solid', heightM: 72 * IN,
      poly: rect(hubX - hubHalf, cy - hubHalf, hubX + hubHalf, cy + hubHalf),
      cite: '[GM] 47x47in; center from tags 18-21,24-27',
    },
    {
      id: 'bump-left', name: 'BUMP', type: 'bump', heightM: 6.513 * IN,
      poly: rect(hubX - bumpD / 2, bumpY0, hubX + bumpD / 2, bumpY0 + bumpW),
      cite: '[GM] 73.0x44.4in, beside HUB',
    },
    {
      id: 'bump-right', name: 'BUMP', type: 'bump', heightM: 6.513 * IN,
      poly: rect(hubX - bumpD / 2, cy - hubHalf - bumpW, hubX + bumpD / 2, cy - hubHalf),
      cite: '[GM] 73.0x44.4in, beside HUB',
    },
    {
      id: 'trench-left', name: 'TRENCH', type: 'low-clearance', heightM: 22.25 * IN,
      poly: rect(hubX - trenchD / 2, bumpY0 + bumpW, hubX + trenchD / 2, W),
      cite: '[GM] 65.65x47.0in, guardrail→BUMP; tags 22/23',
    },
    {
      id: 'trench-right', name: 'TRENCH', type: 'low-clearance', heightM: 22.25 * IN,
      poly: rect(hubX - trenchD / 2, 0, hubX + trenchD / 2, cy - hubHalf - bumpW),
      cite: '[GM] 65.65x47.0in, guardrail→BUMP; tags 17/28',
    },
    {
      // Open uprights at the wall + a floor plate: robots can't drive through it
      // (solid for collision warnings), but cameras see tags 31/32 (15/16) on
      // its wall face — so it must not block sightlines.
      id: 'tower', name: 'TOWER', type: 'solid', blocksVision: false, heightM: 78.25 * IN,
      poly: rect(0, towerY - (39.0 * IN) / 2, 45.18 * IN, towerY + (39.0 * IN) / 2),
      cite: '[GM] base 39.0x45.18in; center from tags 31/32',
    },
    {
      id: 'outpost', name: 'OUTPOST', type: 'marking',
      poly: rect(0, outpostY - (35.8 * IN) / 2, 0.15, outpostY + (35.8 * IN) / 2),
      cite: '[GM] corral 35.8in wide; center from tags 29/30',
    },
    {
      id: 'depot', name: 'DEPOT', type: 'marking', approx: true,
      poly: rect(0, depotCY - (42 * IN) / 2, 27 * IN, depotCY + (42 * IN) / 2),
      cite: '[GM] 42.0x27.0in along alliance wall; Y position approx',
    },
  ];
  return els;
}

const rot180 = (L, W) => ([x, y]) => [+(L - x).toFixed(4), +(W - y).toFixed(4)];
const round = ([x, y]) => [+x.toFixed(4), +y.toFixed(4)];

const out = {
  game: '2026 REBUILT',
  generatedBy: 'sim-ui/scripts/gen-field.mjs',
  units: 'meters, WPILib blue origin',
  symmetry: 'rotational',
  // [GM] ALLIANCE ZONE 158.6in deep; ROBOT STARTING LINE at its edge
  startingLineFromWallM: +(158.6 * IN).toFixed(4),
  autoDurationS: 20,       // [GM] 7.4.1 "first 20 seconds of the MATCH"
  fuelPreloadMax: 8,       // [GM] "8 FUEL may be preloaded in each ROBOT"
  hubOpeningHeightM: +(72 * IN).toFixed(4),
  robotRules: {            // [GM] section 8
    maxPerimeterM: +(110.0 * IN).toFixed(4),  // R104
    maxHeightM: +(30.0 * IN).toFixed(4),      // R104 / R107
    maxBumperExtensionM: +(4.0 * IN).toFixed(4), // R403
    minBumperFoamM: +(2.25 * IN).toFixed(4),  // R402-A
  },
  variants: {},
};

for (const [name, v] of Object.entries(VARIANTS)) {
  const { length: L, width: W } = v;
  const blue = blueElements(v);
  const elements = [
    ...blue.map((e) => ({ ...e, alliance: 'blue', id: `blue-${e.id}`, poly: e.poly.map(round) })),
    ...blue.map((e) => ({ ...e, alliance: 'red', id: `red-${e.id}`, poly: e.poly.map(rot180(L, W)) })),
  ];
  out.variants[name] = {
    length: L,
    width: W,
    hubCenters: {
      blue: [+mid(v.hubX).toFixed(4), +mid(v.hubY).toFixed(4)],
      red: [+(L - mid(v.hubX)).toFixed(4), +(W - mid(v.hubY)).toFixed(4)],
    },
    elements,
  };
}

const json = JSON.stringify(out, null, 2) + '\n';
const targets = [
  resolve(here, '../src/field/field-2026.json'),
  resolve(here, '../../Swerve9520 ctre tuner x project (og home base ver.)/src/main/deploy/sim/field-2026.json'),
];
for (const t of targets) {
  mkdirSync(dirname(t), { recursive: true });
  writeFileSync(t, json);
  console.log('wrote', t);
}
