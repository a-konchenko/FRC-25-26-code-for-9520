// Properties of the open path (right sidebar).
import type { Collision } from '../../pathplanner/collision';
import { deleteWaypoint, insertWaypoint, moveAnchor, normDeg } from '../../pathplanner/edit';
import type { PPConstraints, PPEventMarker, PPPath } from '../../pathplanner/types';
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
          <h3>Event markers</h3>
          <button onClick={() => {
            const pos = selWp >= 0 ? selWp : n / 2;
            const m: PPEventMarker = { name: 'Marker', waypointRelativePos: pos, endWaypointRelativePos: null, command: allNames[0] ? { type: 'named', data: { name: allNames[0] } } : null };
            update({ ...path, eventMarkers: [...path.eventMarkers, m] });
            setSel({ type: 'marker', i: path.eventMarkers.length });
          }}>＋ Add</button>
        </div>
        {path.eventMarkers.length === 0 && <p className="muted tiny">Run a named command while driving (e.g. SpinUp, IntakeOn). Drivetrain commands like AutoAim can't run here — put them as auto steps.</p>}
        {path.eventMarkers.map((m, k) => (
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
