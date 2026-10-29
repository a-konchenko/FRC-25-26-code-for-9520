// PathPlanner file formats (.path / .auto, version "2025.x" — what
// PathPlannerLib 2026.1.2 reads). Index signatures keep fields this editor
// doesn't know about, so saving never drops data written by the PathPlanner app.

export interface XY { x: number; y: number; }

export interface PPWaypoint {
  anchor: XY;
  prevControl: XY | null;
  nextControl: XY | null;
  isLocked: boolean;
  linkedName: string | null;
  [k: string]: unknown;
}

export interface PPRotationTarget {
  waypointRelativePos: number;
  rotationDegrees: number;
  [k: string]: unknown;
}

export type PPCommandType = 'named' | 'wait' | 'path' | 'sequential' | 'parallel' | 'race' | 'deadline';

export interface PPCommand {
  type: PPCommandType;
  data: { name?: string; waitTime?: number; pathName?: string; commands?: PPCommand[]; [k: string]: unknown };
}

export interface PPEventMarker {
  name: string;
  waypointRelativePos: number;
  endWaypointRelativePos: number | null;
  command: PPCommand | null;
  [k: string]: unknown;
}

export interface PPConstraints {
  maxVelocity: number;            // m/s
  maxAcceleration: number;        // m/s²
  maxAngularVelocity: number;     // deg/s
  maxAngularAcceleration: number; // deg/s²
  nominalVoltage: number;
  unlimited: boolean;
  [k: string]: unknown;
}

/** A PathPlanner constraint zone: inside [min, max] these constraints replace the path's own (first matching zone wins). */
export interface PPConstraintZone {
  name: string;
  minWaypointRelativePos: number;
  maxWaypointRelativePos: number;
  constraints: PPConstraints;
  [k: string]: unknown;
}

export interface PPState { velocity: number; rotation: number; [k: string]: unknown; } // m/s, degrees

export interface PPPath {
  version: string;
  waypoints: PPWaypoint[];
  rotationTargets: PPRotationTarget[];
  constraintZones: unknown[];
  pointTowardsZones: unknown[];
  eventMarkers: PPEventMarker[];
  globalConstraints: PPConstraints;
  goalEndState: PPState;
  reversed: boolean;
  folder: string | null;
  idealStartingState: PPState;
  useDefaultConstraints: boolean;
  [k: string]: unknown;
}

export interface PPAuto {
  version: string;
  command: PPCommand;
  resetOdom: boolean;
  folder: string | null;
  choreoAuto: boolean;
  [k: string]: unknown;
}

export interface PPSettings {
  robotWidth: number;
  robotLength: number;
  holonomicMode: boolean;
  defaultMaxVel: number;
  defaultMaxAccel: number;
  defaultMaxAngVel: number;
  defaultMaxAngAccel: number;
  defaultNominalVoltage: number;
  robotMass: number;
  robotMOI: number;
  robotTrackwidth: number;
  driveWheelRadius: number;
  driveGearing: number;
  maxDriveSpeed: number;
  driveMotorType: string;
  driveCurrentLimit: number;
  wheelCOF: number;
  flModuleX: number; flModuleY: number;
  frModuleX: number; frModuleY: number;
  blModuleX: number; blModuleY: number;
  brModuleX: number; brModuleY: number;
  [k: string]: unknown;
}

export const FILE_VERSION = '2025.0';

export function defaultConstraints(s?: Partial<PPSettings>): PPConstraints {
  return {
    maxVelocity: s?.defaultMaxVel ?? 3.0,
    maxAcceleration: s?.defaultMaxAccel ?? 3.0,
    maxAngularVelocity: s?.defaultMaxAngVel ?? 540,
    maxAngularAcceleration: s?.defaultMaxAngAccel ?? 720,
    nominalVoltage: s?.defaultNominalVoltage ?? 12.0,
    unlimited: false,
  };
}

/** A new two-waypoint path from a to b (straight, controls at thirds). */
export function newPath(a: XY, b: XY, settings?: Partial<PPSettings>): PPPath {
  const d = { x: (b.x - a.x) / 3, y: (b.y - a.y) / 3 };
  return {
    version: FILE_VERSION,
    waypoints: [
      { anchor: a, prevControl: null, nextControl: { x: a.x + d.x, y: a.y + d.y }, isLocked: false, linkedName: null },
      { anchor: b, prevControl: { x: b.x - d.x, y: b.y - d.y }, nextControl: null, isLocked: false, linkedName: null },
    ],
    rotationTargets: [],
    constraintZones: [],
    pointTowardsZones: [],
    eventMarkers: [],
    globalConstraints: defaultConstraints(settings),
    goalEndState: { velocity: 0, rotation: 0 },
    reversed: false,
    folder: null,
    idealStartingState: { velocity: 0, rotation: 0 },
    useDefaultConstraints: true,
  };
}

export function newAuto(): PPAuto {
  return {
    version: FILE_VERSION,
    command: { type: 'sequential', data: { commands: [] } },
    resetOdom: true,
    folder: null,
    choreoAuto: false,
  };
}

/** Path names an auto uses, in the order they run (depth-first). */
export function autoPathNames(cmd: PPCommand | undefined): string[] {
  if (!cmd) return [];
  if (cmd.type === 'path' && cmd.data.pathName) return [cmd.data.pathName];
  return (cmd.data.commands ?? []).flatMap(autoPathNames);
}

/** The same command tree with every `path` step pointing at `from` renamed to `to` (other fields kept). */
export function renamePathIn(cmd: PPCommand, from: string, to: string): PPCommand {
  if (cmd.type === 'path') return cmd.data.pathName === from ? { ...cmd, data: { ...cmd.data, pathName: to } } : cmd;
  if (!cmd.data.commands) return cmd;
  return { ...cmd, data: { ...cmd.data, commands: cmd.data.commands.map((c) => renamePathIn(c, from, to)) } };
}

/** Named commands an auto or path uses (for "unknown name" warnings). */
export function commandNames(cmd: PPCommand | null | undefined): string[] {
  if (!cmd) return [];
  if (cmd.type === 'named' && cmd.data.name) return [cmd.data.name];
  return (cmd.data.commands ?? []).flatMap(commandNames);
}

/**
 * Replace every `path` step for `from` with `steps` (a path split by Stop &
 * wait). Inside a sequential group the steps go inline; anywhere else they are
 * wrapped in a sequential group, so a parallel/race/deadline keeps its meaning.
 */
export function expandPathIn(cmd: PPCommand, from: string, steps: PPCommand[]): PPCommand {
  const isTarget = (c: PPCommand) => c.type === 'path' && c.data.pathName === from;
  if (isTarget(cmd)) return { type: 'sequential', data: { commands: steps } };
  if (!cmd.data.commands) return cmd;
  const out: PPCommand[] = [];
  for (const c of cmd.data.commands) {
    if (isTarget(c)) {
      if (cmd.type === 'sequential') out.push(...steps);
      else out.push({ type: 'sequential', data: { commands: steps } });
    } else out.push(expandPathIn(c, from, steps));
  }
  return { ...cmd, data: { ...cmd.data, commands: out } };
}

/** Where an auto stops and waits: a path step directly followed by a wait step (in a sequential group). */
export function waitsAfterPaths(cmd: PPCommand): { pathName: string; seconds: number }[] {
  const cs = cmd.data.commands ?? [];
  const out: { pathName: string; seconds: number }[] = [];
  cs.forEach((c, j) => {
    const next = cs[j + 1];
    if (cmd.type === 'sequential' && c.type === 'path' && c.data.pathName && next?.type === 'wait') {
      out.push({ pathName: c.data.pathName, seconds: next.data.waitTime ?? 0 });
    }
    out.push(...waitsAfterPaths(c));
  });
  return out;
}
