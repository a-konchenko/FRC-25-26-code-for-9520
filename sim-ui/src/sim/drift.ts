// Drift settings — the UI is the source of truth; values are sent to
// /Sim/Control/drift/* and the robot sim's DriftModel uses them live.

export interface DriftSettings {
  wheelScalePct: number;       // % the wheels over-report distance
  slipCmPerSqrtM: number;      // random-walk slip
  slipAccelThreshold: number;  // m/s² above which slip is multiplied
  slipAccelMultiplier: number;
  gyroBiasDegPerMin: number;
  gyroScalePct: number;        // % of rotation the gyro over-reports
}

export const DRIFT_PRESETS: Record<string, { label: string; hint: string; s: DriftSettings }> = {
  ideal: {
    label: 'Ideal',
    hint: 'Perfect sensors — truth and estimate never separate.',
    s: { wheelScalePct: 0, slipCmPerSqrtM: 0, slipAccelThreshold: 3, slipAccelMultiplier: 1, gyroBiasDegPerMin: 0, gyroScalePct: 0 },
  },
  welltuned: {
    label: 'Well-tuned',
    hint: 'Good robot, fresh tread: ~10–20 cm off after a 20 s auto.',
    s: { wheelScalePct: 1.5, slipCmPerSqrtM: 1, slipAccelThreshold: 3, slipAccelMultiplier: 3, gyroBiasDegPerMin: 0.3, gyroScalePct: 0.2 },
  },
  worn: {
    label: 'Worn wheels',
    hint: 'Worn tread, dusty carpet: errors roughly double.',
    s: { wheelScalePct: 3, slipCmPerSqrtM: 2, slipAccelThreshold: 2.5, slipAccelMultiplier: 3, gyroBiasDegPerMin: 0.5, gyroScalePct: 0.4 },
  },
  rough: {
    label: 'Rough match',
    hint: 'Defense, hard acceleration, bump crossings (bumps in phase 5).',
    s: { wheelScalePct: 3, slipCmPerSqrtM: 4, slipAccelThreshold: 2, slipAccelMultiplier: 4, gyroBiasDegPerMin: 1, gyroScalePct: 0.5 },
  },
};

export const DRIFT_FIELDS: { key: keyof DriftSettings; label: string; unit: string; min: number; max: number; step: number; help: string }[] = [
  { key: 'wheelScalePct', label: 'Wheel scale error', unit: '%', min: 0, max: 6, step: 0.1,
    help: 'Worn tread / carpet compression: the wheels think they went further than they did.' },
  { key: 'slipCmPerSqrtM', label: 'Random slip', unit: 'cm/√m', min: 0, max: 6, step: 0.1,
    help: 'Random sideways and forward slip. Grows with the square root of distance.' },
  { key: 'slipAccelThreshold', label: 'Hard-accel threshold', unit: 'm/s²', min: 0.5, max: 8, step: 0.1,
    help: 'Above this acceleration, slip is multiplied (wheels break traction).' },
  { key: 'slipAccelMultiplier', label: 'Hard-accel slip ×', unit: '×', min: 1, max: 8, step: 0.5,
    help: 'How much worse slip gets while accelerating hard.' },
  { key: 'gyroBiasDegPerMin', label: 'Gyro bias drift', unit: '°/min', min: 0, max: 3, step: 0.05,
    help: 'Pigeon 2 heading creep over time (only while enabled).' },
  { key: 'gyroScalePct', label: 'Gyro scale error', unit: '%', min: 0, max: 1.5, step: 0.05,
    help: 'Heading error per rotation — spinning in place makes this show up.' },
];

/** Rough expected error after driving `meters` in a straight line. */
export function expectedStraightErrorM(s: DriftSettings, meters: number): number {
  const scale = (s.wheelScalePct / 100) * meters;
  const slip = (s.slipCmPerSqrtM / 100) * Math.sqrt(meters) * Math.SQRT2;
  return Math.hypot(scale, slip);
}
