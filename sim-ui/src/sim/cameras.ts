// Camera settings the UI owns and sends to /Sim/Control/cam/<key>/*.
export type CamKey = 'll3' | 'll4' | 'pv';

export interface CamSettings {
  enabled: boolean;
  noise: number;           // × the noise model
  latencyMs: number;
  dropPct: number;         // % of frames lost (Limelights)
  mountErrX: number;       // m — where the camera really is vs where the code thinks
  mountErrY: number;
  mountErrYawDeg: number;
}

export const CAM_LABEL: Record<CamKey, string> = {
  ll3: 'Limelight 3 — MegaTag2 pose',
  ll4: 'Limelight 4 — shooter aim',
  pv: 'PhotonVision ×4 (Jetson)',
};

export const CAM_HINT: Record<CamKey, string> = {
  ll3: 'Gives the robot its field position from AprilTags (MegaTag2). It uses the heading YOUR CODE sends it, so it cannot fix a wrong heading — a 2° heading error ≈ 9 cm position error at 2.5 m. Ignored while spinning > 360°/s.',
  ll4: 'Sees the hub/trench tags for the pipeline the code selects; reports tx/ty for auto-aim.',
  pv: 'Four Arducam cameras on the Jetson, multi-tag pose. Optional on the real robot.',
};

export const CAM_DEFAULTS: Record<CamKey, CamSettings> = {
  ll3: { enabled: true, noise: 1, latencyMs: 30, dropPct: 0, mountErrX: 0, mountErrY: 0, mountErrYawDeg: 0 },
  ll4: { enabled: true, noise: 1, latencyMs: 20, dropPct: 0, mountErrX: 0, mountErrY: 0, mountErrYawDeg: 0 },
  pv: { enabled: false, noise: 1, latencyMs: 30, dropPct: 0, mountErrX: 0, mountErrY: 0, mountErrYawDeg: 0 },
};

export interface FuelSettings { preload: number; aimTolDeg: number; minDistM: number; maxDistM: number; }
export const FUEL_DEFAULTS: FuelSettings = { preload: 8, aimTolDeg: 3, minDistM: 1.5, maxDistM: 5.5 };

export const SHOT_REASON = ['scored', 'missed — aim off', 'missed — too close', 'missed — too far', 'missed — flywheel too slow'];
