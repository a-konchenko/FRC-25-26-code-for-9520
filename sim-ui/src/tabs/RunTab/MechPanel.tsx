// Mechanisms + FUEL (phase 4).
import { sendAction, useNT } from '../../nt/store';
import { SHOT_REASON, type FuelSettings } from '../../sim/cameras';

interface Props {
  fuel: FuelSettings;
  setFuel: (f: FuelSettings) => void;
}

export function MechPanel({ fuel, setFuel }: Props) {
  // {shooter rps, setpoint, feeder rps, intake rps, arm 0..1, climbL 0..1, climbR 0..1, atSpeed}
  const m = useNT<number[]>('/Sim/Mech/state', []);
  const f = useNT<number[]>('/Sim/Fuel/state', [0, 0, 0, 0]); // {remaining, scored, missed, preload}
  const shots = useNT<number[]>('/Sim/Fuel/shots', []);
  const last = shots.length >= 6 ? shots.slice(-6) : null;
  const sp = Math.abs(m[1] ?? 0);

  return (
    <>
      <section className="panel">
        <h3>Fuel</h3>
        <div className="stat-row">
          <div className="stat"><div className="stat-value">{f[0]}</div><div className="stat-label">in robot</div></div>
          <div className="stat stat-ok"><div className="stat-value">{f[1]}</div><div className="stat-label">scored</div></div>
          <div className={`stat ${f[2] > 0 ? 'stat-bad' : ''}`}><div className="stat-value">{f[2]}</div><div className="stat-label">missed</div></div>
        </div>
        {last && <p className={`small ${last[5] === 0 ? 'ok' : 'warn'}`}>Last shot: {SHOT_REASON[last[5]]}</p>}
        <p className="muted tiny">A ball scores if the robot really points within ±{fuel.aimTolDeg}° of its hub, is {fuel.minDistM}–{fuel.maxDistM} m away and the flywheel is ≥ 90 % of its setpoint. Simple model — no ball flight.</p>
        <label className="slider"><span className="slider-label">Preload</span>
          <input type="range" min={0} max={8} step={1} value={fuel.preload} onChange={(e) => setFuel({ ...fuel, preload: Number(e.target.value) })} />
          <span className="slider-value">{fuel.preload}</span></label>
        <label className="slider"><span className="slider-label">Aim tolerance</span>
          <input type="range" min={0.5} max={10} step={0.5} value={fuel.aimTolDeg} onChange={(e) => setFuel({ ...fuel, aimTolDeg: Number(e.target.value) })} />
          <span className="slider-value">±{fuel.aimTolDeg}°</span></label>
        <label className="slider"><span className="slider-label">Max distance</span>
          <input type="range" min={2} max={8} step={0.25} value={fuel.maxDistM} onChange={(e) => setFuel({ ...fuel, maxDistM: Number(e.target.value) })} />
          <span className="slider-value">{fuel.maxDistM} m</span></label>
        <button className="small-btn" onClick={() => sendAction('resetFuel')}>Reload fuel</button>
      </section>

      <section className="panel">
        <h3>Mechanisms</h3>
        <Gauge label="Flywheel" value={Math.abs(m[0] ?? 0)} max={Math.max(sp * 1.2, 60)} mark={sp} text={`${Math.abs(m[0] ?? 0).toFixed(0)} / ${sp.toFixed(0)} rps${m[7] > 0.5 ? ' ✓' : ''}`} ok={m[7] > 0.5} />
        <Gauge label="Feeder" value={Math.abs(m[2] ?? 0)} max={60} text={`${Math.abs(m[2] ?? 0).toFixed(0)} rps`} />
        <Gauge label="Intake" value={Math.abs(m[3] ?? 0)} max={100} text={`${Math.abs(m[3] ?? 0).toFixed(0)} rps`} />
        <Gauge label="Intake arm" value={m[4] ?? 0} max={1} text={`${((m[4] ?? 0) * 100).toFixed(0)} %`} />
        <Gauge label="Climber L" value={m[5] ?? 0} max={1} text={`${((m[5] ?? 0) * 100).toFixed(0)} %`} />
        <Gauge label="Climber R" value={m[6] ?? 0} max={1} text={`${((m[6] ?? 0) * 100).toFixed(0)} %`} />
        <p className="muted tiny">Motor physics on the simulated TalonFX controllers, driven by the robot code's own commands. Gearing, inertia and travel are placeholders (TO MEASURE).</p>
      </section>
    </>
  );
}

function Gauge({ label, value, max, text, mark, ok }: { label: string; value: number; max: number; text: string; mark?: number; ok?: boolean }) {
  const f = Math.min(Math.max(value / max, 0), 1);
  return (
    <div className="gauge">
      <span className="gauge-label small">{label}</span>
      <div className="gauge-bar">
        <div className={`gauge-fill ${ok ? 'ok' : ''}`} style={{ width: `${f * 100}%` }} />
        {mark != null && mark > 0 && <div className="gauge-mark" style={{ left: `${Math.min(mark / max, 1) * 100}%` }} />}
      </div>
      <span className="gauge-text small">{text}</span>
    </div>
  );
}
