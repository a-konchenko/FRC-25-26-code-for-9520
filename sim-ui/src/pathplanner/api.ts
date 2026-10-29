// Client for the dev-server file API (server/deployFiles.ts).
import type { PPAuto, PPPath, PPSettings } from './types';

export interface FileEntry { name: string; mtime: number; }
export interface FileList { root: string; paths: FileEntry[]; autos: FileEntry[]; settings: number | null; }

export async function listFiles(): Promise<FileList> {
  const r = await fetch('/api/pp/list');
  if (!r.ok) throw new Error(`list failed (${r.status})`);
  return r.json();
}

async function readJson<T>(rel: string): Promise<T> {
  const r = await fetch(`/api/pp/file?path=${encodeURIComponent(rel)}`);
  if (!r.ok) throw new Error(`${rel}: ${(await r.json().catch(() => ({}))).error ?? r.status}`);
  return r.json();
}

/** Write a file; returns its new modification time (ms), for the "changed on disk" check. */
async function writeJson(rel: string, data: unknown): Promise<number> {
  // 2-space indent, like the PathPlanner app
  const r = await fetch(`/api/pp/file?path=${encodeURIComponent(rel)}`, { method: 'PUT', body: JSON.stringify(data, null, 2) });
  if (!r.ok) throw new Error(`save ${rel}: ${(await r.json().catch(() => ({}))).error ?? r.status}`);
  return (await r.json()).mtime;
}

async function remove(rel: string): Promise<void> {
  const r = await fetch(`/api/pp/file?path=${encodeURIComponent(rel)}`, { method: 'DELETE' });
  if (!r.ok) throw new Error(`delete ${rel}: ${(await r.json().catch(() => ({}))).error ?? r.status}`);
}

export const readPath = (name: string) => readJson<PPPath>(`paths/${name}.path`);
export const readAuto = (name: string) => readJson<PPAuto>(`autos/${name}.auto`);
export const readSettings = () => readJson<PPSettings>('settings.json');
export const writePath = (name: string, p: PPPath) => writeJson(`paths/${name}.path`, p);
export const writeAuto = (name: string, a: PPAuto) => writeJson(`autos/${name}.auto`, a);
export const writeSettings = (s: PPSettings) => writeJson('settings.json', s);
export const deletePath = (name: string) => remove(`paths/${name}.path`);
export const deleteAuto = (name: string) => remove(`autos/${name}.auto`);

/** Modification time of a file as the server sees it now (null = doesn't exist). */
export async function fileMtime(kind: 'path' | 'auto' | 'settings', name = ''): Promise<number | null> {
  const l = await listFiles();
  if (kind === 'settings') return l.settings;
  return (kind === 'path' ? l.paths : l.autos).find((f) => f.name === name)?.mtime ?? null;
}

/** True if a file with this name exists, ignoring upper/lower case (macOS and Windows treat them as one file). */
export function nameTaken(list: FileEntry[], name: string): boolean {
  return list.some((f) => f.name.toLowerCase() === name.toLowerCase());
}

/** File names can't contain path separators or characters most file systems reject. */
export function validName(name: string): boolean {
  return name.trim().length > 0 && !/[\\/:*?"<>|]/.test(name) && !name.startsWith('.');
}
