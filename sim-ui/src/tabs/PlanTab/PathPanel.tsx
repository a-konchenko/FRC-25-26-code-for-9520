// Properties of the open path (right sidebar).
import type { Collision } from '../../pathplanner/collision';
import {
  STOP_MARGIN, deleteWaypoint, insertWaypoint, isSpeedZone, isStopMarker, maxSpeedAt, moveAnchor, newSpeedZone, newStopMarker, normDeg,
} from '../../pathplanner/edit';
import type { PPConstraintZone, PPConstraints, PPEventMarker, PPPath } from '../../pathplanner/types';
import type { Selection } from './selection';

interface Props {
  path: PPPath;
  sel: Selection;
  setSel: (s: Selection) => void;
  update: (p: PPPath, pushHistory?: boolean) => void;
  collisions: Collision[];
  namedOptions: string[];
  connected: boolean;
  robotLength: number;
}

/** Robot top speed (m/s); a speed marker above this can't be reached. */
const ROBOT_MAX_SPEED = 4.44;

/** Length of a new Stop & wait (seconds); edit it on the marker afterwards. */
const DEFAULT_STOP_S = 3.0;

export function PathPanel({ path, sel, setSel, update, collisions, namedOptions, connected }: Props) {
  const n = path.waypoints.length - 1;
  const selWp = sel && (sel.type === 'anchor' || sel.type === 'prev' || sel.type === 'next') ? sel.i : -1;
  const c = path.globalConstraints;
  const setC = (k: keyof PPConstraints, v: number) =>
    update({ ...path, globalConstraints: { ...c, [k]: v }, useDefaultConstraints: false });

  const usedNames = path.eventMarkers.map((m) => m.command?.data?.name).filter((x): x is string => !!x);
  const unknown = connected ? usedNames.filter((nm) => !namedOptions.includes(nm)) : [];
  const allNames = [...new Set([...namedOptions, ...usedNames])].sort();

  const collisionSummary = summarize(collisions);

  return (
    <>
      {(collisionSummary.length > 0 || unknown.length > 0) && (
        <section className="panel">
          <h3>Warnings</h3>
          {collisionSummary.map((s, k) => <p key={k} className="warn small">⚠ Bumpers hit the {s.what} around waypoint {s.from.toFixed(1)}–{s.to.toFixed(1)}.</p>)}
          {unknown.map((nm) => <p key={nm} className="warn small">⚠ Named command “{nm}” is not registered in RobotContainer — it will do nothing.</p>)}
        </section>
      )}

      <section className="panel">
        <h3>Waypoints</h3>
        <table className="grid">
          <thead><tr><th>#</th><th>x (m)</th><th>y (m)</th></tr></thead>
          <tbody>
            {path.waypoints.map((w, i) => (
              <tr key={i} className={i === selWp ? 'selected' : ''} onClick={() => setSel({ type: 'anchor', i })}>
                <td>{i}</td>
                <td><Num value={w.anchor.x} step={0.05} onChange={(x) => update(moveAnchor(path, i, { x, y: w.anchor.y }))} /></td>
                <td><Num value={w.anchor.y} step={0.05} onChange={(y) => update(moveAnchor(path, i, { x: w.anchor.x, y }))} /></td>
              </tr>
            ))}
          </tbody>
        </table>
        <div className="row-buttons">
          <button disabled={selWp < 0 || selWp >= n} onClick={() => { update(insertWaypoint(path, selWp)); setSel({ type: 'anchor', i: selWp + 1 }); }}>Insert after</button>
          <button disabled={selWp < 0 || n < 2} onClick={() => { update(deleteWaypoint(path, selWp)); setSel(null); }}>Delete</button>
        </div>
        <p className="muted tiny">Drag a handle with Shift to break the smooth curve. Use “＋ Waypoint” in the toolbar to extend the path.</p>
      </section>

      <section className="panel">
        <h3>Start &amp; end</h3>
        <div className="form-grid">
          <label>Start heading (°)</label>
          <Num value={path.idealStartingState.rotation} step={5} onChange={(v) => update({ ...path, idealStartingState: { ...path.idealStartingState, rotation: normDeg(v) } })} />
          <label>Start velocity (m/s)</label>
          <Num value={path.idealStartingState.velocity} step={0.1} min={0} onChange={(v) => update({ ...path, idealStartingState: { ...path.idealStartingState, velocity: v } })} />
          <label>End heading (°)</label>
          <Num value={path.goalEndState.rotation} step={5} onChange={(v) => update({ ...path, goalEndState: { ...path.goalEndState, rotation: normDeg(v) } })} />
          <label>End velocity (m/s)</label>
          <Num value={path.goalEndState.velocity} step={0.1} min={0} onChange={(v) => update({ ...path, goalEndState: { ...path.goalEndState, velocity: v } })} />
        </div>
        <p className="muted tiny">Heading 0° faces the red alliance wall (+X); 90° faces the left side wall (+Y).</p>
      </section>

      <section className="panel">
        <h3>Constraints</h3>
        <div className="form-grid">
          <label>Max velocity (m/s)</label><Num value={c.maxVelocity} step={0.1} min={0.1} onChange={(v) => setC('maxVelocity', v)} />
          <label>Max accel (m/s²)</label><Num value={c.maxAcceleration} step={0.1} min={0.1} onChange={(v) => setC('maxAcceleration', v)} />
          <label>Max turn rate (°/s)</label><Num value={c.maxAngularVelocity} step={10} min={10} onChange={(v) => setC('maxAngularVelocity', v)} />
          <label>Max turn accel (°/s²)</label><Num value={c.maxAngularAcceleration} step={10} min={10} onChange={(v) => setC('maxAngularAcceleration', v)} />
        </div>
        <p className="muted tiny">The robot tops out at 4.44 m/s. Lower limits = slower but more accurate tracking.</p>
      </section>

      <section className="panel">
        <div className="panel-head">
          <h3>Rotation targets</h3>
          <button onClick={() => {
            const pos = selWp >= 0 ? Math.min(selWp + 0.5, n) : n / 2;
            update({ ...path, rotationTargets: [...path.rotationTargets, { waypointRelativePos: pos, rotationDegrees: 0 }] });
            setSel({ type: 'rot', i: path.rotationTargets.length });
          }}>＋ Add</button>
        </div>
        {path.rotationTargets.length === 0 && <p className="muted tiny">Make the robot face a direction at a point along the path (holonomic).</p>}
        {path.rotationTargets.map((r, k) => (
          <div key={k} className={`item ${sel?.type === 'rot' && sel.i === k ? 'selected' : ''}`} onClick={() => setSel({ type: 'rot', i: k })}>
            <label className="slider">
              <span className="slider-label">at waypoint</span>
              <input type="range" min={0} max={n} step={0.01} value={r.waypointRelativePos}
                onChange={(e) => update({ ...path, rotationTargets: path.rotationTargets.map((t, j) => (j === k ? { ...t, waypointRelativePos: Number(e.target.value) } : t)) }, false)} />
              <span className="slider-value">{r.waypointRelativePos.toFixed(2)}</span>
            </label>
            <div className="item-row">
              <span className="small">heading</span>
              <Num value={r.rotationDegrees} step={5} onChange={(v) => update({ ...path, rotationTargets: path.rotationTargets.map((t, j) => (j === k ? { ...t, rotationDegrees: normDeg(v) } : t)) })} />
              <span className="small">°</span>
              <button className="danger small-btn" onClick={(e) => { e.stopPropagation(); update({ ...path, rotationTargets: path.rotationTargets.filter((_, j) => j !== k) }); setSel(null); }}>✕</button>
            </div>
          </div>
        ))}
      </section>

      <section className="panel">
        <div className="panel-head">
          <h3>Speed markers</h3>
          <button onClick={() => {
            const pos = selWp >= 0 && selWp < n ? selWp : Math.round(n * 50) / 100;
            const v = Math.round(maxSpeedAt(path, pos) * 50) / 100; // half the speed there: slow down
            update({ ...path, constraintZones: [...(path.constraintZones ?? []), newSpeedZone(path, pos, v)] });
            setSel({ type: 'speed', i: (path.constraintZones ?? []).length });
          }} title="Change the max speed from this point on">＋ Speed</button>
        </div>
        {!(path.constraintZones ?? []).some(isSpeedZone) && <p className="muted tiny">Slow down (▼) or speed up (▲) from a point on, until the next speed marker or the end of the path. Saved as PathPlanner constraint zones.</p>}
        {(path.constraintZones ?? []).map((z, k) => {
          if (!isSpeedZone(z)) return null;
          const before = z.minWaypointRelativePos <= 1e-9 ? c.maxVelocity : maxSpeedAt(path, z.minWaypointRelativePos - 1e-3);
          const up = z.constraints.maxVelocity > before + 1e-9;
          const setZone = (patch: Partial<PPConstraintZone>, history = true) => update({
            ...path, constraintZones: (path.constraintZones ?? []).map((x, j) => (j === k ? { ...(x as PPConstraintZone), ...patch } : x)),
          }, history);
          const setZC = (key: keyof PPConstraints, v: number) => setZone({ constraints: { ...z.constraints, [key]: v } });
          return (
            <div key={k} className={`item speed-item ${up ? 'up' : 'down'} ${sel?.type === 'speed' && sel.i === k ? 'selected' : ''}`} onClick={() => setSel({ type: 'speed', i: k })}>
              <div className="item-row">
                <strong className="grow small">{up ? '▲ Speed up' : '▼ Slow down'}</strong>
                <button className="danger small-btn" onClick={(e) => { e.stopPropagation(); update({ ...path, constraintZones: (path.constraintZones ?? []).filter((_, j) => j !== k) }); setSel(null); }}>✕</button>
              </div>
              <div className="form-grid">
                <label>Max velocity (m/s)</label><Num value={z.constraints.maxVelocity} step={0.1} min={0.1} onChange={(v) => setZC('maxVelocity', Math.max(0.1, v))} />
                <label>Max accel (m/s²)</label><Num value={z.constraints.maxAcceleration} step={0.1} min={0.1} onChange={(v) => setZC('maxAcceleration', Math.max(0.1, v))} />
              </div>
              <label className="slider">
                <span className="slider-label">from waypoint</span>
                <input type="range" min={0} max={n - 0.01} step={0.01} value={z.minWaypointRelativePos}
                  onChange={(e) => setZone({ minWaypointRelativePos: Number(e.target.value) }, false)} />
                <span className="slider-value">{z.minWaypointRelativePos.toFixed(2)}</span>
              </label>
              <p className="muted tiny">Until waypoint {z.maxWaypointRelativePos.toFixed(2)}{z.maxWaypointRelativePos >= n - 1e-9 ? ' (end of path)' : ' (next speed marker)'}; before it: {before.toFixed(2)} m/s.</p>
              {z.constraints.maxVelocity > ROBOT_MAX_SPEED && <p className="warn tiny">⚠ The robot tops out at {ROBOT_MAX_SPEED} m/s.</p>}
            </div>
          );
        })}
      </section>

      <section className="panel">
        <div className="panel-head">
          <h3>Event markers</h3>
          <button onClick={() => {
            const pos = Math.min(Math.max(selWp >= 0 ? selWp : n / 2, STOP_MARGIN), n - STOP_MARGIN);
            update({ ...path, eventMarkers: [...path.eventMarkers, newStopMarker(pos, DEFAULT_STOP_S)] });
            setSel({ type: 'marker', i: path.eventMarkers.length });
          }} title="The robot stops here, waits, then drives on">＋ Stop &amp; wait</button>
          <button onClick={() => {
            const pos = selWp >= 0 ? selWp : n / 2;
            const m: PPEventMarker = { name: 'Marker', waypointRelativePos: pos, endWaypointRelativePos: null, command: allNames[0] ? { type: 'named', data: { name: allNames[0] } } : null };
            update({ ...path, eventMarkers: [...path.eventMarkers, m] });
            setSel({ type: 'marker', i: path.eventMarkers.length });
          }}>＋ Add</button>
        </div>
        {path.eventMarkers.length === 0 && <p className="muted tiny">Run a named command while driving (e.g. SpinUp, IntakeOn). Drivetrain commands like AutoAim can't run here — put them as auto steps.</p>}
        {path.eventMarkers.some(isStopMarker) && <p className="muted tiny">⏸ PathPlanner can't pause inside a path, so <b>Save</b> splits the path at each Stop &amp; wait (into “… 2”, “… 3”) and puts a wait step between the parts in the autos that use it.</p>}
        {path.eventMarkers.map((m, k) => isStopMarker(m) ? (
          <div key={k} className={`item stop-item ${sel?.type === 'marker' && sel.i === k ? 'selected' : ''}`} onClick={() => setSel({ type: 'marker', i: k })}>
            <div className="item-row">
              <strong className="grow small">⏸ Stop &amp; wait</strong>
              <Num value={m.command?.data.waitTime ?? 0} step={0.5} min={0}
                onChange={(v) => update({ ...path, eventMarkers: path.eventMarkers.map((x, j) => (j === k ? { ...x, command: { type: 'wait', data: { ...x.command?.data, waitTime: Math.max(0, v) } } } : x)) })} />
              <span className="small">s</span>
              <button className="danger small-btn" onClick={(e) => { e.stopPropagation(); update({ ...path, eventMarkers: path.eventMarkers.filter((_, j) => j !== k) }); setSel(null); }}>✕</button>
            </div>
            <label className="slider">
              <span className="slider-label">at waypoint</span>
              <input type="range" min={STOP_MARGIN} max={n - STOP_MARGIN} step={0.01} value={m.waypointRelativePos}
                onChange={(e) => update({ ...path, eventMarkers: path.eventMarkers.map((x, j) => (j === k ? { ...x, waypointRelativePos: Number(e.target.value) } : x)) }, false)} />
              <span className="slider-value">{m.waypointRelativePos.toFixed(2)}</span>
            </label>
          </div>
        ) : (
          <div key={k} className={`item ${sel?.type === 'marker' && sel.i === k ? 'selected' : ''}`} onClick={() => setSel({ type: 'marker', i: k })}>
            <div className="item-row">
              <input className="grow" value={m.name} onChange={(e) => update({ ...path, eventMarkers: path.eventMarkers.map((x, j) => (j === k ? { ...x, name: e.target.value } : x)) }, false)} />
              <button className="danger small-btn" onClick={(e) => { e.stopPropagation(); update({ ...path, eventMarkers: path.eventMarkers.filter((_, j) => j !== k) }); setSel(null); }}>✕</button>
            </div>
            <label className="slider">
              <span className="slider-label">at waypoint</span>
              <input type="range" min={0} max={n} step={0.01} value={m.waypointRelativePos}
                onChange={(e) => update({ ...path, eventMarkers: path.eventMarkers.map((x, j) => (j === k ? { ...x, waypointRelativePos: Number(e.target.value) } : x)) }, false)} />
              <span className="slider-value">{m.waypointRelativePos.toFixed(2)}</span>
            </label>
            <div className="item-row">
              <span className="small">runs</span>
              <select className="grow" value={m.command?.data?.name ?? ''} onChange={(e) => update({ ...path, eventMarkers: path.eventMarkers.map((x, j) => (j === k ? { ...x, command: e.target.value ? { type: 'named', data: { name: e.target.value } } : null } : x)) })}>
                <option value="">(nothing)</option>
                {allNames.map((nm) => <option key={nm} value={nm}>{nm}</option>)}
              </select>
            </div>
          </div>
        ))}
        {!connected && <p className="muted tiny">Start the robot sim to list the registered named commands.</p>}
      </section>
    </>
  );
}

function summarize(cs: Collision[]): { what: string; from: number; to: number }[] {
  const out: { what: string; from: number; to: number }[] = [];
  for (const c of cs) {
    const last = out[out.length - 1];
    if (last && last.what === c.what && c.pos - last.to < 0.3) last.to = c.pos;
    else out.push({ what: c.what, from: c.pos, to: c.pos });
  }
  return out;
}

/** Number input that commits on change; shows 3 decimals at most. */
export function Num({ value, onChange, step = 0.1, min, max }: { value: number; onChange: (v: number) => void; step?: number; min?: number; max?: number }) {
  return (
    <input type="number" className="num" value={Number.isFinite(value) ? +value.toFixed(3) : 0} step={step} min={min} max={max}
      onChange={(e) => { const v = Number(e.target.value); if (Number.isFinite(v)) onChange(v); }} />
  );
}
