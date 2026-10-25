// Load the "plan" for something the Run tab can play: a single path, or every
// path an auto uses (in order). Used to draw the plan before/while it runs.
import { readAuto, readPath } from './api';
import { samplePath, type PathSample } from './geometry';
import { autoPathNames, type PPPath } from './types';

export type RunKind = 0 | 1 | 2; // 0 = PathPlanner auto, 1 = path, 2 = hand-coded

export interface LoadedPlan {
  paths: { name: string; path: PPPath; samples: PathSample[] }[];
  resetOdom: boolean;
  error?: string;
}

export async function loadPlan(kind: RunKind, name: string): Promise<LoadedPlan> {
  if (!name || kind === 2) return { paths: [], resetOdom: false };
  try {
    const names = kind === 1 ? [name] : autoPathNames((await readAuto(name)).command);
    const resetOdom = kind === 1 ? true : (await readAuto(name)).resetOdom;
    const paths = [];
    for (const n of names) {
      const path = await readPath(n);
      paths.push({ name: n, path, samples: samplePath(path, 30) });
    }
    return { paths, resetOdom };
  } catch (e) {
    return { paths: [], resetOdom: false, error: String(e) };
  }
}
