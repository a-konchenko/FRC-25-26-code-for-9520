// Plan tab — PathPlanner-style editor for the robot project's real
// deploy/pathplanner files (.path / .auto / settings.json).
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { FieldView, type FieldPointer } from '../../field/FieldView';
import { RobotShape, Trace } from '../../field/RobotShape';
import { FIELD, type Variant } from '../../field/fieldData';
import { nt, useConnected, useNT } from '../../nt/store';
import {
  deleteAuto, deletePath, listFiles, readAuto, readPath, readSettings, validName,
  writeAuto, writePath, writeSettings, type FileList,
} from '../../pathplanner/api';
import { findCollisions } from '../../pathplanner/collision';
import { appendWaypoint, moveAnchor, moveControl, normDeg } from '../../pathplanner/edit';
import { nearestPos, pointAt, samplePath, pathLength } from '../../pathplanner/geometry';
import { autoPathNames, newAuto, newPath, type PPAuto, type PPPath, type PPSettings } from '../../pathplanner/types';
import type { RunKind } from '../../pathplanner/plan';
import { HANDLE_R, PathLayer, rotationHandle } from './PathLayer';
import { PathPanel } from './PathPanel';
import { AutoPanel } from './AutoPanel';
import { SettingsPanel } from './SettingsPanel';
import { PreviewChart, type PreviewState } from './PreviewChart';
import type { Selection } from './selection';

type Doc =
  | { kind: 'path'; name: string; data: PPPath }
  | { kind: 'auto'; name: string; data: PPAuto }
  | { kind: 'settings'; data: PPSettings };

interface Props {
  variant: Variant;
  onRun: (kind: RunKind, name: string) => void;
}

export function PlanTab({ variant, onRun }: Props) {
  const connected = useConnected();
  const ntNamed = useNT<string[]>('/Sim/NamedCommands', []);
  const ntSize = useNT<number[]>('/Sim/Robot/size', []);
  const fieldV = FIELD.variants[variant];

  const [files, setFiles] = useState<FileList | null>(null);
  const [fileError, setFileError] = useState('');
  const [settings, setSettings] = useState<PPSettings | null>(null);
  const [doc, setDoc] = useState<Doc | null>(null);
  const [savedJson, setSavedJson] = useState('');
  const [past, setPast] = useState<string[]>([]);
  const [future, setFuture] = useState<string[]>([]);
  const [sel, setSel] = useState<Selection>(null);
  const [tool, setTool] = useState<'select' | 'add'>('select');
  const [showFootprint, setShowFootprint] = useState(true);
  const [status, setStatus] = useState('');
  const [preview, setPreview] = useState<PreviewState | null>(null);
  const [autoPaths, setAutoPaths] = useState<{ name: string; path: PPPath }[]>([]);
  const dragging = useRef<Selection>(null);

  const refresh = useCallback(async () => {
    try {
      setFiles(await listFiles());
      setFileError('');
    } catch (e) {
      setFileError(String(e));
    }
  }, []);
  useEffect(() => {
    refresh();
    readSettings().then(setSettings).catch(() => setSettings(null));
    const t = setInterval(refresh, 3000);
    return () => clearInterval(t);
  }, [refresh]);

  const dirty = doc ? JSON.stringify(doc.data) !== savedJson : false;
  const size = useMemo(() => {
    if (settings) return [settings.robotLength, settings.robotWidth, ntSize[2] ?? settings.robotLength - 0.165, ntSize[3] ?? settings.robotWidth - 0.165];
    return ntSize.length >= 4 ? ntSize : [0.851, 0.775, 0.686, 0.61];
  }, [settings, ntSize]);

  // ── open / new / save ─────────────────────────────────────────────────────
  const confirmDiscard = () => !dirty || window.confirm('Discard unsaved changes?');

  const open = async (kind: 'path' | 'auto' | 'settings', name = '') => {
    if (!confirmDiscard()) return;
    try {
      const d: Doc = kind === 'path' ? { kind, name, data: await readPath(name) }
        : kind === 'auto' ? { kind, name, data: await readAuto(name) }
        : { kind, data: await readSettings() };
      setDoc(d);
      setSavedJson(JSON.stringify(d.data));
      setPast([]); setFuture([]); setSel(null); setPreview(null); setStatus('');
    } catch (e) {
      setStatus(`Could not open: ${e}`);
    }
  };

  const save = async (): Promise<boolean> => {
    if (!doc) return false;
    try {
      if (doc.kind === 'path') await writePath(doc.name, doc.data);
      else if (doc.kind === 'auto') await writeAuto(doc.name, doc.data);
      else { await writeSettings(doc.data); setSettings(doc.data); }
      setSavedJson(JSON.stringify(doc.data));
      setStatus(`Saved ${new Date().toLocaleTimeString()}`);
      refresh();
      return true;
    } catch (e) {
      setStatus(String(e));
      return false;
    }
  };

  const createNew = async (kind: 'path' | 'auto') => {
    if (!confirmDiscard()) return;
    const existing = new Set((kind === 'path' ? files?.paths : files?.autos)?.map((f) => f.name.toLowerCase()));
    let base = kind === 'path' ? 'New Path' : 'New Auto', name = base, k = 2;
    while (existing.has(name.toLowerCase())) name = `${base} ${k++}`;
    const input = window.prompt(`Name for the new ${kind}:`, name);
    if (!input) return;
    if (!validName(input) || existing.has(input.toLowerCase())) { setStatus(`"${input}" is not a valid new name`); return; }
    const W = fieldV.width;
    const data = kind === 'path' ? newPath({ x: 2.5, y: W / 2 + 1.5 }, { x: 6.5, y: W / 2 + 2.6 }, settings ?? undefined) : newAuto();
    if (kind === 'path') await writePath(input, data as PPPath); else await writeAuto(input, data as PPAuto);
    await refresh();
    const d = kind === 'path' ? { kind, name: input, data: data as PPPath } as Doc : { kind, name: input, data: data as PPAuto } as Doc;
    setDoc(d); setSavedJson(JSON.stringify(d.data)); setPast([]); setFuture([]); setSel(null); setPreview(null);
  };

  const renameDoc = async () => {
    if (!doc || doc.kind === 'settings') return;
    const input = window.prompt('New name:', doc.name);
    if (!input || input === doc.name) return;
    if (!validName(input)) { setStatus(`"${input}" is not a valid name`); return; }
    if (doc.kind === 'path') {
      await writePath(input, doc.data);
      await deletePath(doc.name);
      setStatus(`Renamed. Autos that use "${doc.name}" must be updated to "${input}".`);
    } else {
      await writeAuto(input, doc.data);
      await deleteAuto(doc.name);
    }
    setDoc({ ...doc, name: input } as Doc);
    setSavedJson(JSON.stringify(doc.data));
    refresh();
  };

  const duplicateDoc = async () => {
    if (!doc || doc.kind === 'settings') return;
    const input = window.prompt('Name for the copy:', `${doc.name} copy`);
    if (!input || !validName(input)) return;
    if (doc.kind === 'path') await writePath(input, doc.data); else await writeAuto(input, doc.data);
    await refresh();
    setDoc({ ...doc, name: input } as Doc);
    setSavedJson(JSON.stringify(doc.data));
  };

  const deleteDoc = async () => {
    if (!doc || doc.kind === 'settings') return;
    if (!window.confirm(`Delete ${doc.kind} "${doc.name}"? (a backup is kept in sim-ui/.sim-backup)`)) return;
    if (doc.kind === 'path') await deletePath(doc.name); else await deleteAuto(doc.name);
    setDoc(null); setSavedJson(''); refresh();
  };

  // ── editing with undo ─────────────────────────────────────────────────────
  // docRef always holds the latest doc, so several edits between renders
  // (fast drags) build on each other and history is recorded exactly once.
  const docRef = useRef<Doc | null>(null);
  docRef.current = doc;
  const update = useCallback((data: Doc['data'], pushHistory = true) => {
    const d = docRef.current;
    if (!d) return;
    if (pushHistory) { setPast((p) => [...p.slice(-99), JSON.stringify(d.data)]); setFuture([]); }
    const next = { ...d, data } as Doc;
    docRef.current = next;
    setDoc(next);
    if (pushHistory) setPreview(null);
  }, []);
  const undo = () => {
    if (!doc || !past.length) return;
    setFuture((f) => [JSON.stringify(doc.data), ...f]);
    setDoc({ ...doc, data: JSON.parse(past[past.length - 1]) } as Doc);
    setPast((p) => p.slice(0, -1));
  };
  const redo = () => {
    if (!doc || !future.length) return;
    setPast((p) => [...p, JSON.stringify(doc.data)]);
    setDoc({ ...doc, data: JSON.parse(future[0]) } as Doc);
    setFuture((f) => f.slice(1));
  };

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement)?.closest('input, textarea, select')) return;
      const mod = e.metaKey || e.ctrlKey;
      if (mod && e.key.toLowerCase() === 'z') { e.preventDefault(); e.shiftKey ? redo() : undo(); }
      else if (mod && e.key.toLowerCase() === 's') { e.preventDefault(); save(); }
      else if (e.key === 'Escape') { setSel(null); setTool('select'); }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  });

  // ── path geometry for the open path ───────────────────────────────────────
  const path = doc?.kind === 'path' ? doc.data : null;
  const samples = useMemo(() => (path ? samplePath(path) : []), [path]);
  const collisions = useMemo(
    () => (path ? findCollisions(samples, size[0], size[1], fieldV.elements, fieldV.length, fieldV.width) : []),
    [path, samples, size, fieldV]);

  // auto: load the paths it uses (for drawing)
  const auto = doc?.kind === 'auto' ? doc.data : null;
  const autoPathKey = auto ? autoPathNames(auto.command).join('\u0000') : '';
  useEffect(() => {
    if (!autoPathKey) { setAutoPaths([]); return; }
    let cancelled = false;
    Promise.all(autoPathKey.split('\u0000').map(async (n) => ({ name: n, path: await readPath(n) })))
      .then((ps) => { if (!cancelled) setAutoPaths(ps); })
      .catch(() => { if (!cancelled) setAutoPaths([]); });
    return () => { cancelled = true; };
  }, [autoPathKey]);

  // ── pointer: hit test + drag ──────────────────────────────────────────────
  const hit = (x: number, y: number): Selection => {
    if (!path) return null;
    const near = (pt: { x: number; y: number } | null, r = HANDLE_R * 1.6) => !!pt && Math.hypot(pt.x - x, pt.y - y) <= r;
    const wps = path.waypoints, n = wps.length - 1;
    const selWp = sel && (sel.type === 'anchor' || sel.type === 'prev' || sel.type === 'next') ? sel.i : -1;
    if (selWp >= 0) {
      if (near(wps[selWp].prevControl)) return { type: 'prev', i: selWp };
      if (near(wps[selWp].nextControl)) return { type: 'next', i: selWp };
    }
    for (let i = 0; i <= n; i++) if (near(wps[i].anchor)) return { type: 'anchor', i };
    const deg = (d: number) => (d * Math.PI) / 180;
    if (near(rotationHandle(wps[0].anchor, deg(path.idealStartingState.rotation), size[0]))) return { type: 'start' };
    if (near(rotationHandle(wps[n].anchor, deg(path.goalEndState.rotation), size[0]))) return { type: 'end' };
    for (let k = 0; k < path.rotationTargets.length; k++) {
      const r = path.rotationTargets[k];
      const at = pointAt(path, r.waypointRelativePos);
      if (near(rotationHandle(at, deg(r.rotationDegrees), size[0]))) return { type: 'rotHandle', i: k };
      if (near(at)) return { type: 'rot', i: k };
    }
    for (let k = 0; k < path.eventMarkers.length; k++) {
      if (near(pointAt(path, path.eventMarkers[k].waypointRelativePos), 0.15)) return { type: 'marker', i: k };
    }
    return null;
  };

  const onDown = (p: FieldPointer) => {
    if (!path) return;
    const h = hit(p.x, p.y);
    if (h) {
      setSel(h.type === 'rotHandle' ? { type: 'rot', i: h.i } : h);
      dragging.current = h;
      update(path, true); // snapshot for undo before the drag
      return;
    }
    if (tool === 'add') {
      update(appendWaypoint(path, { x: p.x, y: p.y }));
      setSel({ type: 'anchor', i: path.waypoints.length });
      return;
    }
    setSel(null);
  };

  const onMove = (p: FieldPointer) => {
    const d = dragging.current;
    if (!d || !path) return;
    const at = { x: p.x, y: p.y };
    const angleTo = (from: { x: number; y: number }) => normDeg((Math.atan2(p.y - from.y, p.x - from.x) * 180) / Math.PI);
    const snap = (deg: number) => (p.shift ? deg : Math.round(deg / 5) * 5);
    let next: PPPath = path;
    if (d.type === 'anchor') next = moveAnchor(path, d.i, at);
    else if (d.type === 'prev' || d.type === 'next') next = moveControl(path, d.i, d.type, at, p.shift);
    else if (d.type === 'start') next = { ...path, idealStartingState: { ...path.idealStartingState, rotation: snap(angleTo(path.waypoints[0].anchor)) } };
    else if (d.type === 'end') next = { ...path, goalEndState: { ...path.goalEndState, rotation: snap(angleTo(path.waypoints[path.waypoints.length - 1].anchor)) } };
    else if (d.type === 'rotHandle') {
      const r = path.rotationTargets[d.i];
      next = { ...path, rotationTargets: path.rotationTargets.map((t, k) => (k === d.i ? { ...t, rotationDegrees: snap(angleTo(pointAt(path, r.waypointRelativePos))) } : t)) };
    } else if (d.type === 'rot') {
      const pos = Math.round(nearestPos(path, p.x, p.y).pos * 100) / 100;
      next = { ...path, rotationTargets: path.rotationTargets.map((t, k) => (k === d.i ? { ...t, waypointRelativePos: pos } : t)) };
    } else if (d.type === 'marker') {
      const pos = Math.round(nearestPos(path, p.x, p.y).pos * 100) / 100;
      next = { ...path, eventMarkers: path.eventMarkers.map((m, k) => (k === d.i ? { ...m, waypointRelativePos: pos } : m)) };
    }
    if (next !== path) update(next, false);
  };

  const onUp = () => { dragging.current = null; };

  // ── preview ───────────────────────────────────────────────────────────────
  const previewId = useRef(0);
  const previewResult = useNT<number[]>('/Sim/Plan/previewResult');
  const previewError = useNT<string>('/Sim/Plan/previewError', '');
  const requestPreview = () => {
    if (!path) return;
    if (connected) {
      previewId.current = Math.floor(Math.random() * 1e9);
      nt.set('/Sim/Plan/previewRequest', 'string', JSON.stringify({ id: previewId.current, path }));
      setPreview({ exact: true, pending: true, total: 0, states: [], start: performance.now() });
    } else {
      // approximate: trapezoid profile along the curve at the path's max velocity/acceleration
      const vmax = path.globalConstraints.maxVelocity, amax = path.globalConstraints.maxAcceleration;
      const L = pathLength(samples);
      const tAcc = vmax / amax, dAcc = 0.5 * amax * tAcc * tAcc;
      const total = L > 2 * dAcc ? 2 * tAcc + (L - 2 * dAcc) / vmax : 2 * Math.sqrt(L / amax);
      const states: number[][] = [];
      let dist = 0;
      for (let i = 0; i < samples.length; i++) {
        if (i > 0) dist += Math.hypot(samples[i].x - samples[i - 1].x, samples[i].y - samples[i - 1].y);
        const t = timeAtDistance(dist, L, vmax, amax);
        states.push([t, samples[i].x, samples[i].y, samples[i].rot, velocityAtDistance(dist, L, vmax, amax)]);
      }
      setPreview({ exact: false, pending: false, total, states, start: performance.now() });
    }
  };
  useEffect(() => {
    if (!preview?.pending || !previewResult || previewResult[0] !== previewId.current) return;
    const states: number[][] = [];
    for (let k = 2; k + 4 < previewResult.length; k += 5) states.push(previewResult.slice(k, k + 5));
    setPreview({ exact: true, pending: false, total: previewResult[1], states, start: performance.now() });
  }, [previewResult, preview?.pending]);
  useEffect(() => {
    if (preview?.pending && previewError.startsWith(`${previewId.current}:`)) {
      setPreview(null);
      setStatus(`Preview failed: ${previewError.slice(previewError.indexOf(':') + 1)}`);
    }
  }, [previewError, preview?.pending]);

  const [now, setNow] = useState(performance.now());
  useEffect(() => {
    if (!preview || preview.pending) return;
    let raf = 0;
    const tick = () => { setNow(performance.now()); raf = requestAnimationFrame(tick); };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [preview]);
  const previewT = preview && !preview.pending && preview.total > 0 ? ((now - preview.start) / 1000) % (preview.total + 1) : 0;
  const ghost = preview && !preview.pending ? stateAt(preview.states, Math.min(previewT, preview.total)) : null;

  // ── run ───────────────────────────────────────────────────────────────────
  const runThis = async () => {
    if (!doc || doc.kind === 'settings') return;
    if (dirty && !(await save())) return;
    onRun(doc.kind === 'path' ? 1 : 0, doc.name);
  };

  const namedOptions = useMemo(() => [...new Set(ntNamed)].sort(), [ntNamed]);
  const robotLen = size[0];

  return (
    <div className="tab-body">
      <aside className="filelist">
        <section>
          <div className="filelist-head"><h3>Paths</h3><button onClick={() => createNew('path')} title="New path">＋</button></div>
          {files?.paths.map((f) => (
            <button key={f.name} className={`file ${doc?.kind === 'path' && doc.name === f.name ? 'active' : ''}`} onClick={() => open('path', f.name)}>
              {f.name}
            </button>
          ))}
          {files && !files.paths.length && <p className="muted small">No paths yet.</p>}
        </section>
        <section>
          <div className="filelist-head"><h3>Autos</h3><button onClick={() => createNew('auto')} title="New auto">＋</button></div>
          {files?.autos.map((f) => (
            <button key={f.name} className={`file ${doc?.kind === 'auto' && doc.name === f.name ? 'active' : ''}`} onClick={() => open('auto', f.name)}>
              {f.name}
            </button>
          ))}
        </section>
        <section>
          <button className={`file ${doc?.kind === 'settings' ? 'active' : ''}`} onClick={() => open('settings')}>⚙ Robot settings</button>
        </section>
        {fileError && <p className="warn small">File API unavailable: {fileError}. Start the UI with <code>npm run dev</code>.</p>}
        {files && <p className="muted tiny">Files: {files.root.split('/').slice(-4).join('/')}</p>}
      </aside>

      <div className="field-col">
        <div className="toolbar">
          {doc && doc.kind !== 'settings' ? (
            <>
              <strong className="doc-title">{doc.name}{dirty ? ' •' : ''}</strong>
              <span className="muted small">{doc.kind === 'path' ? '.path' : '.auto'}</span>
              <button onClick={save} disabled={!dirty} title="Save (Ctrl/Cmd+S)">Save</button>
              <button onClick={undo} disabled={!past.length} title="Undo (Ctrl/Cmd+Z)">↶</button>
              <button onClick={redo} disabled={!future.length} title="Redo (Shift+Ctrl/Cmd+Z)">↷</button>
              <span className="toolbar-sep" />
              {doc.kind === 'path' && (
                <>
                  <button className={tool === 'add' ? 'active' : ''} onClick={() => setTool(tool === 'add' ? 'select' : 'add')} title="Click on the field to add a waypoint at the end">＋ Waypoint</button>
                  <button onClick={requestPreview} title={connected ? 'Exact timing from PathPlannerLib (same code as the robot)' : 'Approximate timing (start the robot sim for exact)'}>
                    {preview?.pending ? 'Computing…' : '◷ Preview'}
                  </button>
                  {preview && <button onClick={() => setPreview(null)}>✕</button>}
                  <label className="toggle small"><input type="checkbox" checked={showFootprint} onChange={(e) => setShowFootprint(e.target.checked)} />footprint</label>
                </>
              )}
              <span className="toolbar-sep" />
              <button className="primary" onClick={runThis} disabled={!connected} title={connected ? 'Save, switch to Run and play it' : 'Start the robot sim to run'}>▶ Run this</button>
              <span className="toolbar-grow" />
              <button onClick={duplicateDoc}>Duplicate</button>
              <button onClick={renameDoc}>Rename</button>
              <button onClick={deleteDoc} className="danger">Delete</button>
            </>
          ) : (
            <span className="muted">{doc?.kind === 'settings' ? 'Robot settings — deploy/pathplanner/settings.json' : 'Pick a path or auto on the left, or create one with ＋.'}</span>
          )}
        </div>
        {status && <div className="status small">{status}</div>}

        <div className="field-wrap">
          <FieldView variant={variant} onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp}
            cursor={tool === 'add' ? 'crosshair' : undefined}>
            {path && <PathLayer path={path} samples={samples} sel={sel} size={size} collisions={collisions} showFootprint={showFootprint && !preview} />}
            {auto && autoPaths.map((ap, k) => {
              const s = samplePath(ap.path, 30);
              return (
                <g key={k}>
                  <Trace points={s.map((q) => [q.x, q.y])} className={`plan-curve auto-seg auto-seg-${k % 4}`} />
                  {k === 0 && <RobotShape pose={[s[0].x, s[0].y, s[0].rot]} size={size} style="plan-start" />}
                </g>
              );
            })}
            {ghost && <RobotShape pose={[ghost[1], ghost[2], ghost[3]]} size={size} style="preview" />}
          </FieldView>
          {tool === 'add' && <div className="field-banner">Click to add a waypoint at the end of the path · Esc to stop</div>}
        </div>

        {preview && !preview.pending && <PreviewChart preview={preview} t={previewT} />}
      </div>

      <aside className="sidebar">
        {doc?.kind === 'path' && (
          <PathPanel path={doc.data} sel={sel} setSel={setSel} update={update} collisions={collisions}
            namedOptions={namedOptions} connected={connected} robotLength={robotLen} />
        )}
        {doc?.kind === 'auto' && (
          <AutoPanel auto={doc.data} update={update} pathNames={files?.paths.map((f) => f.name) ?? []}
            namedOptions={namedOptions} connected={connected} />
        )}
        {doc?.kind === 'settings' && <SettingsPanel settings={doc.data} update={update} simSize={ntSize} />}
        {!doc && (
          <section className="panel">
            <h3>How it works</h3>
            <ul className="small">
              <li>These are the robot project's real PathPlanner files (<code>src/main/deploy/pathplanner</code>). The PathPlanner app opens the same files.</li>
              <li>Drag waypoints (circles) and their handles. Drag the small dots in front of the robot outlines to set heading.</li>
              <li>Event markers (◆) run a named command while driving. Steps in an auto run one after another.</li>
              <li>Red dots mean the bumpers would hit something.</li>
              <li><b>◷ Preview</b> shows timing; <b>▶ Run this</b> runs it through the real robot code.</li>
            </ul>
          </section>
        )}
      </aside>
    </div>
  );
}

function stateAt(states: number[][], t: number): number[] | null {
  if (!states.length) return null;
  let i = 0;
  while (i < states.length - 1 && states[i + 1][0] < t) i++;
  const a = states[i], b = states[Math.min(i + 1, states.length - 1)];
  const span = b[0] - a[0];
  const f = span > 1e-9 ? Math.min(Math.max((t - a[0]) / span, 0), 1) : 0;
  let dr = b[3] - a[3];
  while (dr > Math.PI) dr -= 2 * Math.PI;
  while (dr < -Math.PI) dr += 2 * Math.PI;
  return [t, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, a[3] + dr * f, a[4] + (b[4] - a[4]) * f];
}

function timeAtDistance(d: number, L: number, v: number, a: number): number {
  const dAcc = Math.min(v * v / (2 * a), L / 2);
  const vPeak = Math.sqrt(2 * a * dAcc);
  if (d <= dAcc) return Math.sqrt(2 * d / a);
  const tAcc = vPeak / a;
  if (d <= L - dAcc) return tAcc + (d - dAcc) / vPeak;
  const rem = L - d;
  const total = 2 * tAcc + (L - 2 * dAcc) / vPeak;
  return total - Math.sqrt(2 * rem / a);
}

function velocityAtDistance(d: number, L: number, v: number, a: number): number {
  return Math.min(v, Math.sqrt(2 * a * Math.max(d, 0)), Math.sqrt(2 * a * Math.max(L - d, 0)));
}
