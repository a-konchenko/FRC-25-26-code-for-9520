// Run tab — pick an auto/path, Play it through the real robot code, and see
// plan vs fact: PLAN (the path file), SETPOINT (where the path follower wants
// the robot now), ESTIMATE (where the code thinks it is) and TRUTH (where it
// really is). Plus drift controls, placement error, recording + timeline.
import { useEffect, useMemo, useRef, useState } from 'react';
import { FieldView, type FieldPointer } from '../../field/FieldView';
import { RobotShape, Trace } from '../../field/RobotShape';
import { FIELD, type Variant } from '../../field/fieldData';
import { ntHost, sendAction, setSetting, useConnected, useNT, useNTFrame, type Pose } from '../../nt/store';
import { DRIFT_FIELDS, DRIFT_PRESETS, expectedStraightErrorM, type DriftSettings } from '../../sim/drift';
import { samplesForAlliance } from '../../pathplanner/geometry';
import { loadPlan, type LoadedPlan, type RunKind } from '../../pathplanner/plan';
import { listFiles } from '../../pathplanner/api';
import { ErrorChart, type ErrorSample } from './ErrorChart';
import { RunChart, locErr, trackingErr, type RunFrame } from './RunChart';
import { load, save } from '../../util/storage';
import type { RunRequest } from '../../App';
import { CAM_DEFAULTS, FUEL_DEFAULTS, type CamKey, type CamSettings, type FuelSettings } from '../../sim/cameras';
import { VisionLayer } from './VisionLayer';
import { CameraPanel } from './CameraPanel';
import { MechPanel } from './MechPanel';
import { FuelLayer } from './FuelLayer';

type Tool = 'none' | 'place' | 'kidnap';
interface Layers { plan: boolean; setpoint: boolean; truth: boolean; estimate: boolean; traces: boolean; tags: boolean; cameras: boolean; vision: boolean; shots: boolean; fuel: boolean; }
interface Placement { x: number; y: number; deg: number; } // placement error, cm / cm / degrees

const KIND_LABEL: Record<RunKind, string> = { 0: 'Auto', 1: 'Path', 2: 'Hand-coded' };

export function RunTab({ variant, runRequest }: { variant: Variant; runRequest: RunRequest | null }) {
  const connected = useConnected();
  const frame = useNTFrame();
  const fieldV = FIELD.variants[variant];

  // ── live robot state ──────────────────────────────────────────────────────
  // truth + estimate arrive together (same robot loop) — see /Sim/TruthEstimate
  const pair = useNT<number[]>('/Sim/TruthEstimate', []);
  const truth: Pose | undefined = pair.length >= 6 ? [pair[0], pair[1], pair[2]] : undefined;
  const estimate: Pose | undefined = pair.length >= 6 ? [pair[3], pair[4], pair[5]] : undefined;
  const setpointActive = pair.length >= 10 && pair[6] > 0.5;
  const setpoint: Pose | undefined = pair.length >= 10 ? [pair[7], pair[8], pair[9]] : undefined;
  const activePath = useNT<number[]>('/Sim/ActivePath', []);
  const size = useNT<number[]>('/Sim/Robot/size', []);
  const moduleLocations = useNT<number[]>('/Sim/Robot/moduleLocations', []);
  const modules = useNT<number[]>('/Sim/Modules', []);
  const maxSpeed = useNT<number>('/Sim/Robot/maxSpeedMps', 4.44);
  const driftState = useNT<number[]>('/Sim/Drift/state', [0, 0, 0, 0]);
  const mode = useNT<string>('/Sim/Match/mode', '—');
  const simTime = useNT<number>('/Sim/Time', 0);
  const paused = useNT<boolean>('/Sim/Clock/paused', false);
  const runState = useNT<string>('/Sim/Run/state', 'idle');
  const runId = useNT<number>('/Sim/Run/id', 0);
  const runElapsed = useNT<number>('/Sim/Run/elapsed', 0);
  const runCommandEnd = useNT<number>('/Sim/Run/commandEnd', -1);
  const runDuration = useNT<number>('/Sim/Run/duration', FIELD.autoDurationS);
  const runError = useNT<string>('/Sim/Run/error', '');
  const ntAutos = useNT<string[]>('/Sim/Autos/pathplanner', []);
  const ntPaths = useNT<string[]>('/Sim/Autos/paths', []);
  const ntHand = useNT<string[]>('/Sim/Autos/handCoded', []);

  // ── settings (persisted per browser) ──────────────────────────────────────
  const [drift, setDrift] = useState<DriftSettings>(() => load('drift', DRIFT_PRESETS.welltuned.s));
  const [seed, setSeed] = useState<number>(() => load('seed', 9520));
  const [layers, setLayers] = useState<Layers>(() => load('runLayers2', { plan: true, setpoint: true, truth: true, estimate: true, traces: true, tags: true, cameras: true, vision: true, shots: true, fuel: true }));
  // Merge per camera, so a setting added later still gets its default
  const [cams, setCams] = useState<Record<CamKey, CamSettings>>(() => {
    const stored = load('cams', CAM_DEFAULTS);
    return Object.fromEntries((Object.keys(CAM_DEFAULTS) as CamKey[]).map((k) => [k, { ...CAM_DEFAULTS[k], ...stored[k] }])) as Record<CamKey, CamSettings>;
  });
  const [fuel, setFuel] = useState<FuelSettings>(() => load('fuel', FUEL_DEFAULTS));
  useEffect(() => {
    save('cams', cams);
    for (const [key, c] of Object.entries(cams)) {
      for (const [k, v] of Object.entries(c)) setSetting(`cam/${key}/${k}`, typeof v === 'boolean' ? (v ? 1 : 0) : v);
    }
  }, [cams]);
  useEffect(() => { save('fuel', fuel); for (const [k, v] of Object.entries(fuel)) setSetting(`fuel/${k}`, v); }, [fuel]);
  const fuelState = useNT<number[]>('/Sim/Fuel/state', [0, 0, 0, 0]);
  const intakeRunning = useNT<boolean>('/SmartDashboard/Intake/Running', false);
  const [kind, setKind] = useState<RunKind>(() => load('runKind', 1 as RunKind));
  const [names, setNames] = useState<Record<RunKind, string>>(() => load('runNames', { 0: '', 1: 'Test path', 2: 'Shoot All (Stationary)' }));
  const [red, setRed] = useState<boolean>(() => load('runRed', false));
  const [placement, setPlacement] = useState<Placement>(() => load('placement', { x: 0, y: 0, deg: 0 }));
  useEffect(() => { save('drift', drift); for (const [k, v] of Object.entries(drift)) setSetting(`drift/${k}`, v); }, [drift]);
  useEffect(() => { save('seed', seed); setSetting('seed', seed); }, [seed]);
  useEffect(() => save('runLayers2', layers), [layers]);
  useEffect(() => save('runKind', kind), [kind]);
  useEffect(() => save('runNames', names), [names]);
  useEffect(() => save('runRed', red), [red]);
  useEffect(() => save('placement', placement), [placement]);

  // lists: from the robot sim when connected, else from the files
  const [fileLists, setFileLists] = useState<{ autos: string[]; paths: string[] }>({ autos: [], paths: [] });
  useEffect(() => {
    if (connected) return;
    listFiles().then((l) => setFileLists({ autos: l.autos.map((f) => f.name), paths: l.paths.map((f) => f.name) })).catch(() => {});
  }, [connected]);
  const options: Record<RunKind, string[]> = {
    0: connected ? ntAutos : fileLists.autos,
    1: connected ? ntPaths : fileLists.paths,
    2: connected ? ntHand : ['Shoot All (Stationary)', 'Do Nothing', 'Drive Forward 5s'],
  };
  const name = names[kind] && options[kind].includes(names[kind]) ? names[kind] : options[kind][0] ?? '';

  // ── plan (from the files) ─────────────────────────────────────────────────
  const [plan, setPlan] = useState<LoadedPlan>({ paths: [], resetOdom: false });
  const [planNonce, setPlanNonce] = useState(0);
  useEffect(() => {
    let cancelled = false;
    loadPlan(kind, name).then((p) => { if (!cancelled) setPlan(p); });
    return () => { cancelled = true; };
  }, [kind, name, planNonce]);
  const planSamples = useMemo(
    () => plan.paths.map((p) => samplesForAlliance(p.samples, red, fieldV.length, fieldV.width)),
    [plan, red, fieldV]);

  // ── recording / replay state ──────────────────────────────────────────────
  const frames = useRef<RunFrame[]>([]);
  const recordedRun = useRef(-1);
  const [scrubT, setScrubT] = useState<number | null>(null);

  // ── play / reset ──────────────────────────────────────────────────────────
  const placementArgs = () => [placement.x / 100, placement.y / 100, (placement.deg * Math.PI) / 180];
  const play = (k: RunKind = kind, n: string = name) => {
    if (!n) return;
    setSetting('run/name', n);
    sendAction('play', [k, red ? 1 : 0, ...placementArgs()]);
    setPlanNonce((x) => x + 1);
    setScrubT(null);
  };
  const reset = () => {
    setSetting('run/name', name);
    sendAction('reset', [kind, red ? 1 : 0, ...placementArgs()]);
    setPlanNonce((x) => x + 1);
    setScrubT(null);
  };
  // "▶ Run this" from the Plan tab
  const lastRequest = useRef(0);
  useEffect(() => {
    if (!runRequest || runRequest.nonce === lastRequest.current) return;
    lastRequest.current = runRequest.nonce;
    setKind(runRequest.kind);
    setNames((ns) => ({ ...ns, [runRequest.kind]: runRequest.name }));
    play(runRequest.kind, runRequest.name);
  }, [runRequest]); // eslint-disable-line react-hooks/exhaustive-deps

  // ── recording: one frame per robot loop while the auto runs ───────────────
  useEffect(() => {
    if (runId !== recordedRun.current && runState !== 'finished') {
      frames.current = [];
      recordedRun.current = runId;
    }
    if (runState !== 'running' || !truth || !estimate) return;
    const fs = frames.current;
    if (fs.length && runElapsed <= fs[fs.length - 1].t) return;
    fs.push({ t: runElapsed, truth, est: estimate, sp: setpointActive && setpoint ? setpoint : null });
  }, [frame]); // eslint-disable-line react-hooks/exhaustive-deps

  const replay = scrubT != null ? frameAt(frames.current, scrubT) : null;

  // ── traces + live error history (test drives / manual driving) ───────────
  const truthTrace = useRef<[number, number][]>([]);
  const estTrace = useRef<[number, number][]>([]);
  const errHistory = useRef<ErrorSample[]>([]);
  const lastPose = useRef<{ t?: Pose; e?: Pose }>({});
  const [, force] = useState(0);
  const clearTraces = () => {
    truthTrace.current = []; estTrace.current = []; errHistory.current = [];
    force((n) => n + 1);
  };
  useEffect(() => { if (runState === 'armed') clearTraces(); }, [runState]);
  useEffect(() => {
    if (!truth || !estimate) return;
    const push = (arr: [number, number][], p: Pose, last?: Pose) => {
      const d = last ? Math.hypot(p[0] - last[0], p[1] - last[1]) : Infinity;
      if (last && d > 0.5) { arr.length = 0; arr.push([p[0], p[1]]); return true; } // teleport
      if (d > 0.01) { arr.push([p[0], p[1]]); if (arr.length > 6000) arr.shift(); return true; }
      return false;
    };
    if (push(truthTrace.current, truth, lastPose.current.t)) lastPose.current.t = truth;
    if (push(estTrace.current, estimate, lastPose.current.e)) lastPose.current.e = estimate;
    const h = errHistory.current;
    if (!h.length || simTime - h[h.length - 1].t >= 0.05) {
      h.push({ t: simTime, loc: driftState[2], head: driftState[3] });
      while (h.length && simTime - h[0].t > 60) h.shift();
    }
  }, [frame]); // eslint-disable-line react-hooks/exhaustive-deps

  // ── place / kidnap ────────────────────────────────────────────────────────
  const [tool, setTool] = useState<Tool>('none');
  const [drag, setDrag] = useState<{ x: number; y: number; th: number; free: boolean } | null>(null);
  const onDown = (p: FieldPointer) => {
    if (tool === 'none') return;
    setDrag({ x: p.x, y: p.y, th: (tool === 'kidnap' ? truth?.[2] : estimate?.[2]) ?? 0, free: p.shift });
  };
  const onMove = (p: FieldPointer) => {
    if (!drag) return;
    const dx = p.x - drag.x, dy = p.y - drag.y;
    if (Math.hypot(dx, dy) > 0.15) setDrag({ ...drag, th: Math.atan2(dy, dx), free: p.shift });
  };
  const onUp = () => {
    if (!drag) return;
    const th = drag.free ? drag.th : snapAngle(drag.th);
    if (tool === 'place') { sendAction('placeRobot', [drag.x, drag.y, th]); clearTraces(); }
    else if (tool === 'kidnap') sendAction('kidnap', [drag.x, drag.y, th]);
    setDrag(null); setTool('none');
  };

  // ── summary of the recorded run ───────────────────────────────────────────
  const summary = summarize(frames.current);
  const running = runState === 'running' || runState === 'armed' || runState === 'disarming';
  const shownT = scrubT ?? (running || runState === 'finished' ? runElapsed : 0);
  const locErrCm = driftState[2] * 100;
  const activePts: [number, number][] = [];
  for (let i = 0; i + 2 < activePath.length; i += 3) activePts.push([activePath[i], activePath[i + 1]]);
  const replayUpTo = scrubT != null ? frames.current.filter((f) => f.t <= scrubT) : [];

  return (
    <div className="tab-body">
      <div className="field-col">
        <div className="toolbar runbar">
          <span className={`pill ${mode === 'disabled' ? '' : 'pill-live'} ${paused ? 'pill-paused' : ''}`}>{connected ? (paused ? 'PAUSED' : mode.toUpperCase()) : 'NO ROBOT SIM'}</span>
          <div className="seg">
            {([0, 1, 2] as RunKind[]).map((k) => (
              <button key={k} className={kind === k ? 'active' : ''} onClick={() => setKind(k)}>{KIND_LABEL[k]}</button>
            ))}
          </div>
          <select className="run-name" value={name} onChange={(e) => setNames({ ...names, [kind]: e.target.value })}>
            {options[kind].length === 0 && <option value="">(none)</option>}
            {options[kind].map((n) => <option key={n} value={n}>{n}</option>)}
          </select>
          <div className="seg">
            <button className={!red ? 'active seg-blue' : ''} onClick={() => setRed(false)}>Blue</button>
            <button className={red ? 'active seg-red' : ''} onClick={() => setRed(true)}>Red</button>
          </div>
          <button className="primary" onClick={() => play()} disabled={!connected || !name} title="Build it fresh from the files, place the robot, run the 20 s autonomous period">▶ Play</button>
          <button onClick={reset} disabled={!connected || !name} title="Stop and put the robot on the start pose">⟲ Reset</button>
          <button onClick={() => sendAction('stop')} disabled={!connected}>■ Stop</button>
          <span className="toolbar-sep" />
          {paused ? (
            <>
              <button className="active" onClick={() => sendAction('clock', [0])}>▶ Resume</button>
              <button onClick={() => sendAction('clock', [2, 0.02])} title="Advance one 20 ms robot loop">⏭ 20 ms</button>
            </>
          ) : (
            <button onClick={() => sendAction('clock', [1])} disabled={!connected} title="Freeze robot time">⏸ Pause</button>
          )}
          <span className="run-clock">
            {scrubT != null && <span className="replay-tag">REPLAY</span>}
            t = {shownT.toFixed(2)} / {runDuration.toFixed(0)} s
          </span>
        </div>
        <div className="toolbar toolbar-sub">
          <button className={tool === 'place' ? 'active' : ''} onClick={() => setTool(tool === 'place' ? 'none' : 'place')}>Place robot</button>
          <button className={tool === 'kidnap' ? 'active' : ''} onClick={() => setTool(tool === 'kidnap' ? 'none' : 'kidnap')} title="Move only the TRUE robot — the code doesn't know">Kidnap</button>
          <button onClick={() => { sendAction('syncTruth'); clearTraces(); }}>Sync truth</button>
          <span className="toolbar-sep" />
          <button onClick={() => { clearTraces(); sendAction('testDrive', [1]); }} disabled={!connected}>Test: square</button>
          <button onClick={() => { clearTraces(); sendAction('testDrive', [2]); }} disabled={!connected}>Test: spin</button>
          <button className={intakeRunning ? 'active' : ''} onClick={() => sendAction('intake', [intakeRunning ? 0 : 1])} disabled={!connected}
            title="Turn the robot code's intake on/off (what the X button does). The roller only spins while the robot is enabled.">Intake {intakeRunning ? 'on' : 'off'}</button>
          <span className="toolbar-grow" />
          {scrubT != null && <button onClick={() => setScrubT(null)}>● Live</button>}
          <button onClick={clearTraces}>Clear traces</button>
        </div>
        {runError && <div className="status warn small">Could not build: {runError}</div>}
        {plan.error && kind !== 2 && <div className="status warn small">{plan.error}</div>}

        <div className="field-wrap">
          <FieldView variant={variant} showTagIds={layers.tags} onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp}
            cursor={tool !== 'none' ? 'crosshair' : undefined}>
            {layers.plan && planSamples.map((s, k) => (
              <Trace key={k} points={s.map((q) => [q.x, q.y])} className={`trace trace-plan ${running && activePts.length ? 'dim' : ''}`} />
            ))}
            {layers.plan && planSamples[0]?.length > 0 && !running && scrubT == null && (
              <RobotShape pose={[planSamples[0][0].x, planSamples[0][0].y, planSamples[0][0].rot]} size={size} style="plan-start" title="Planned start pose" />
            )}
            {layers.plan && activePts.length > 1 && <Trace points={activePts} className="trace trace-active" />}

            {scrubT == null && <FuelLayer truth={truth} size={size} showFuel={layers.fuel} />}
            {replay ? (
              <>
                {layers.traces && <Trace points={replayUpTo.map((f) => [f.est[0], f.est[1]])} className="trace trace-estimate" />}
                {layers.traces && <Trace points={replayUpTo.map((f) => [f.truth[0], f.truth[1]])} className="trace trace-truth" />}
                {layers.setpoint && replay.sp && <RobotShape pose={replay.sp} size={size} style="setpoint" title="SETPOINT at this moment" />}
                {layers.estimate && <RobotShape pose={replay.est} size={size} style="estimate" />}
                {layers.truth && <RobotShape pose={replay.truth} size={size} style="truth" alliance={red ? 'red' : 'blue'} />}
              </>
            ) : (
              <>
                {layers.traces && <Trace points={estTrace.current} className="trace trace-estimate" />}
                {layers.traces && <Trace points={truthTrace.current} className="trace trace-truth" />}
                {layers.setpoint && setpointActive && setpoint && <RobotShape pose={setpoint} size={size} style="setpoint" title="SETPOINT — where the path follower wants the robot right now" />}
                {layers.estimate && estimate && <RobotShape pose={estimate} size={size} style="estimate" title="ESTIMATE — where the robot code thinks it is" />}
                {layers.truth && truth && (
                  <RobotShape pose={truth} size={size} style="truth" alliance={red ? 'red' : 'blue'} moduleLocations={moduleLocations} modules={modules} maxSpeed={maxSpeed}
                    title="TRUTH — where the robot really is" />
                )}
              </>
            )}
            {scrubT == null && (
              <VisionLayer variant={variant} truth={truth} showCameras={layers.cameras} showVision={layers.vision} showShots={layers.shots} />
            )}
            {drag && <RobotShape pose={[drag.x, drag.y, drag.free ? drag.th : snapAngle(drag.th)]} size={size} style="placing" />}
          </FieldView>
          {tool !== 'none' && (
            <div className="field-banner">{tool === 'place' ? 'Click to place the robot (estimate + truth). Drag to aim. Shift = free angle.' : 'Click where the robot REALLY is. The robot code is not told.'}</div>
          )}
        </div>

        {frames.current.length > 0 || running
          ? <RunChart frames={frames.current} duration={runDuration} cursor={scrubT ?? (running ? runElapsed : null)} onScrub={setScrubT} />
          : <ErrorChart samples={errHistory.current} />}
      </div>

      <aside className="sidebar">
        <section className="panel">
          <h3>Layers</h3>
          <Toggle label={<><i className="swatch swatch-plan" />Plan — the path file</>} checked={layers.plan} onChange={(v) => setLayers({ ...layers, plan: v })} />
          <Toggle label={<><i className="swatch swatch-setpoint" />Setpoint — where it should be now</>} checked={layers.setpoint} onChange={(v) => setLayers({ ...layers, setpoint: v })} />
          <Toggle label={<><i className="swatch swatch-estimate" />Estimate — what the code believes</>} checked={layers.estimate} onChange={(v) => setLayers({ ...layers, estimate: v })} />
          <Toggle label={<><i className="swatch swatch-truth" />Truth — where it really is</>} checked={layers.truth} onChange={(v) => setLayers({ ...layers, truth: v })} />
          <Toggle label={<><i className="swatch swatch-cam" />Camera views + sightlines</>} checked={layers.cameras} onChange={(v) => setLayers({ ...layers, cameras: v })} />
          <Toggle label={<><i className="swatch swatch-fused" />Vision measurements used</>} checked={layers.vision} onChange={(v) => setLayers({ ...layers, vision: v })} />
          <Toggle label={<><i className="swatch swatch-shot" />Fuel shots</>} checked={layers.shots} onChange={(v) => setLayers({ ...layers, shots: v })} />
          <Toggle label={<><i className="swatch swatch-fuel" />FUEL on the field + intake</>} checked={layers.fuel} onChange={(v) => setLayers({ ...layers, fuel: v })} />
          <Toggle label="Traces" checked={layers.traces} onChange={(v) => setLayers({ ...layers, traces: v })} />
          <Toggle label="AprilTag IDs" checked={layers.tags} onChange={(v) => setLayers({ ...layers, tags: v })} />
        </section>

        <section className="panel">
          <h3>{summary ? 'Last run' : 'Run'}</h3>
          {summary ? (
            <>
              <div className="stat-row">
                <Stat label="Max tracking" value={summary.maxTrack != null ? `${(summary.maxTrack * 100).toFixed(1)} cm` : '—'} tone={tone(summary.maxTrack, 0.1, 0.25)} />
                <Stat label="Max localization" value={`${(summary.maxLoc * 100).toFixed(1)} cm`} tone={tone(summary.maxLoc, 0.05, 0.15)} />
              </div>
              <div className="stat-row">
                <Stat label="Final localization" value={`${(summary.finalLoc * 100).toFixed(1)} cm`} tone={tone(summary.finalLoc, 0.05, 0.15)} />
                <Stat label="Auto finished" value={runCommandEnd >= 0 ? `${runCommandEnd.toFixed(1)} s` : running ? '…' : 'not in time'} tone={runCommandEnd >= 0 ? 'ok' : running ? undefined : 'bad'} />
              </div>
              <div className="stat-row">
                <Stat label="Fuel scored" value={`${fuelState[1]} / ${fuelState[3]}`} tone={fuelState[1] > 0 ? 'ok' : undefined} />
                <Stat label="Fuel missed" value={`${fuelState[2]}`} tone={fuelState[2] > 0 ? 'bad' : undefined} />
              </div>
              <p className="muted tiny"><b>Tracking</b> = setpoint vs estimate → the path follower (tuning, too-fast paths). <b>Localization</b> = estimate vs truth → drift, placement and cameras.</p>
            </>
          ) : (
            <p className="muted small">Pick an auto or path above and press <b>▶ Play</b>. The robot code builds it fresh from the files and runs the {runDuration.toFixed(0)} s autonomous period.</p>
          )}
          {kind === 0 && plan.paths.length > 0 && <p className="muted tiny">Paths: {plan.paths.map((p) => p.name).join(' → ')}{plan.resetOdom ? '' : ' (no odometry reset)'}</p>}
          {kind === 2 && <p className="muted tiny">Hand-coded autos start wherever the robot is — use Place robot first.</p>}
        </section>

        <CameraPanel cams={cams} setCam={(k, c) => setCams((all) => ({ ...all, [k]: c }))} />
        <MechPanel fuel={fuel} setFuel={setFuel} />

        <section className="panel">
          <h3>Placement error</h3>
          <p className="muted tiny">How far from the planned start the students actually set the robot down. The code assumes the planned pose; the truth starts here.</p>
          <label className="slider"><span className="slider-label">X offset</span>
            <input type="range" min={-30} max={30} step={1} value={placement.x} onChange={(e) => setPlacement({ ...placement, x: Number(e.target.value) })} />
            <span className="slider-value">{placement.x} cm</span></label>
          <label className="slider"><span className="slider-label">Y offset</span>
            <input type="range" min={-30} max={30} step={1} value={placement.y} onChange={(e) => setPlacement({ ...placement, y: Number(e.target.value) })} />
            <span className="slider-value">{placement.y} cm</span></label>
          <label className="slider"><span className="slider-label">Heading offset</span>
            <input type="range" min={-5} max={5} step={0.5} value={placement.deg} onChange={(e) => setPlacement({ ...placement, deg: Number(e.target.value) })} />
            <span className="slider-value">{placement.deg}°</span></label>
          <div className="row-buttons">
            <button onClick={() => setPlacement({ x: 0, y: 0, deg: 0 })}>Perfect</button>
            <button onClick={() => setPlacement({ x: rnd(10), y: rnd(10), deg: Math.round(rnd(4)) / 2 })}>Random (±10 cm)</button>
          </div>
        </section>

        <section className="panel">
          <h3>Localization now</h3>
          <div className="stat-row">
            <Stat label="Position" value={`${locErrCm.toFixed(1)} cm`} tone={locErrCm > 15 ? 'bad' : locErrCm > 5 ? 'warn' : 'ok'} />
            <Stat label="Heading" value={`${driftState[3].toFixed(2)}°`} tone={Math.abs(driftState[3]) > 2 ? 'bad' : Math.abs(driftState[3]) > 0.5 ? 'warn' : 'ok'} />
          </div>
        </section>

        <section className="panel">
          <h3>Drift</h3>
          <div className="preset-row">
            {Object.entries(DRIFT_PRESETS).map(([k, p]) => (
              <button key={k} className={sameDrift(drift, p.s) ? 'active' : ''} onClick={() => setDrift(p.s)} title={p.hint}>{p.label}</button>
            ))}
          </div>
          {DRIFT_FIELDS.map((f) => (
            <label key={f.key} className="slider" title={f.help}>
              <span className="slider-label">{f.label}</span>
              <input type="range" min={f.min} max={f.max} step={f.step} value={drift[f.key]} onChange={(e) => setDrift({ ...drift, [f.key]: Number(e.target.value) })} />
              <span className="slider-value">{drift[f.key].toFixed(f.step < 0.1 ? 2 : 1)} {f.unit}</span>
            </label>
          ))}
          <label className="slider">
            <span className="slider-label">Random seed</span>
            <input type="number" value={seed} onChange={(e) => setSeed(Number(e.target.value) || 0)} />
            <span className="slider-value muted">same seed = same run</span>
          </label>
          <p className="muted small">≈ {(expectedStraightErrorM(drift, 10) * 100).toFixed(0)} cm expected after a 10 m straight drive (no cameras).</p>
        </section>

        <section className="panel">
          <h3>Robot</h3>
          <p className="muted small">
            {size.length >= 4
              ? `${(size[0] * 39.37).toFixed(1)} × ${(size[1] * 39.37).toFixed(1)} in with bumpers. Placeholder — TO MEASURE in SimRobotConstants.java.`
              : 'Waiting for the robot sim…'}
          </p>
          {!connected && <p className="warn small">Not connected to {ntHost}:5810 — start the robot sim (./sim.sh).</p>}
        </section>
      </aside>
    </div>
  );
}

function frameAt(fs: RunFrame[], t: number): RunFrame | null {
  if (!fs.length) return null;
  let lo = 0, hi = fs.length - 1;
  while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (fs[mid].t <= t) lo = mid; else hi = mid - 1; }
  return fs[lo];
}

function summarize(fs: RunFrame[]) {
  if (!fs.length) return null;
  let maxTrack: number | null = null, maxLoc = 0;
  for (const f of fs) {
    const tr = trackingErr(f);
    if (tr != null) maxTrack = Math.max(maxTrack ?? 0, tr);
    maxLoc = Math.max(maxLoc, locErr(f));
  }
  return { maxTrack, maxLoc, finalLoc: locErr(fs[fs.length - 1]) };
}

function tone(v: number | null, warn: number, bad: number): 'ok' | 'warn' | 'bad' | undefined {
  if (v == null) return undefined;
  return v > bad ? 'bad' : v > warn ? 'warn' : 'ok';
}

const rnd = (r: number) => Math.round((Math.random() * 2 - 1) * r);

function snapAngle(th: number): number {
  const step = Math.PI / 12;
  return Math.round(th / step) * step;
}

function sameDrift(a: DriftSettings, b: DriftSettings) {
  return (Object.keys(b) as (keyof DriftSettings)[]).every((k) => Math.abs(a[k] - b[k]) < 1e-9);
}

function Toggle({ label, checked, onChange }: { label: React.ReactNode; checked: boolean; onChange: (v: boolean) => void }) {
  return (
    <label className="toggle">
      <input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)} />
      <span>{label}</span>
    </label>
  );
}

function Stat({ label, value, tone }: { label: string; value: string; tone?: 'ok' | 'warn' | 'bad' }) {
  return (
    <div className={`stat ${tone ? `stat-${tone}` : ''}`}>
      <div className="stat-value">{value}</div>
      <div className="stat-label">{label}</div>
    </div>
  );
}
