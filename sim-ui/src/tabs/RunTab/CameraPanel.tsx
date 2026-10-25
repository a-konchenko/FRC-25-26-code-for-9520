// Camera controls (phase 3) + what the Limelight 4 sees.
import { useNT } from '../../nt/store';
import { CAM_DEFAULTS, CAM_HINT, CAM_LABEL, type CamKey, type CamSettings } from '../../sim/cameras';

interface Props {
  cams: Record<CamKey, CamSettings>;
  setCam: (key: CamKey, s: CamSettings) => void;
}

export function CameraPanel({ cams, setCam }: Props) {
  const ll3Vis = useNT<number[]>('/Sim/Vision/ll3/visible', []);
  const pvVis = useNT<number[]>('/Sim/Vision/pv/visible', []);
  const sees: Record<CamKey, string> = {
    ll3: ll3Vis.length ? `sees ${ll3Vis.join(', ')}` : 'no tags',
    ll4: '',
    pv: pvVis.length ? `${pvVis.length / 2} tag sightings` : 'no tags',
  };

  return (
    <section className="panel">
      <h3>Cameras</h3>
      <p className="muted tiny">Cameras look from the TRUE robot. Their measurements pull the estimate back to the truth: try <b>Kidnap</b> with a camera on, then off.</p>
      {(Object.keys(CAM_LABEL) as CamKey[]).map((key) => {
        const c = cams[key];
        const set = (patch: Partial<CamSettings>) => setCam(key, { ...c, ...patch });
        const mountOff = c.mountErrX !== 0 || c.mountErrY !== 0 || c.mountErrYawDeg !== 0;
        return (
          <details key={key} className="cam-card" open={key !== 'pv'}>
            <summary>
              <label className="toggle" onClick={(e) => e.stopPropagation()}>
                <input type="checkbox" checked={c.enabled} onChange={(e) => set({ enabled: e.target.checked })} />
                <span>{CAM_LABEL[key]}</span>
              </label>
              <span className="muted tiny">{c.enabled ? sees[key] : 'off'}{mountOff ? ' · mount error!' : ''}</span>
            </summary>
            <p className="muted tiny">{CAM_HINT[key]}</p>
            <Slider label="Noise" unit="×" min={0} max={5} step={0.1} value={c.noise} onChange={(v) => set({ noise: v })} />
            <Slider label="Latency" unit="ms" min={0} max={150} step={5} value={c.latencyMs} onChange={(v) => set({ latencyMs: v })} />
            {key !== 'pv' && <Slider label="Dropped frames" unit="%" min={0} max={90} step={5} value={c.dropPct} onChange={(v) => set({ dropPct: v })} />}
            <div className="mount-err">
              <span className="small" title="The camera is physically somewhere else than the code / Limelight config assumes">Mount error</span>
              <Slider label="X" unit="cm" min={-20} max={20} step={1} value={c.mountErrX * 100} onChange={(v) => set({ mountErrX: v / 100 })} />
              <Slider label="Y" unit="cm" min={-20} max={20} step={1} value={c.mountErrY * 100} onChange={(v) => set({ mountErrY: v / 100 })} />
              <Slider label="Yaw" unit="°" min={-10} max={10} step={0.5} value={c.mountErrYawDeg} onChange={(v) => set({ mountErrYawDeg: v })} />
              {mountOff && <button className="small-btn" onClick={() => set({ mountErrX: 0, mountErrY: 0, mountErrYawDeg: 0 })}>Zero</button>}
            </div>
            {key === 'll4' && <LL4View />}
          </details>
        );
      })}
      <button className="small-btn" onClick={() => (Object.keys(CAM_DEFAULTS) as CamKey[]).forEach((k) => setCam(k, CAM_DEFAULTS[k]))}>Reset cameras</button>
    </section>
  );
}

/** The LL4 image as the robot code sees it: crosshair + target at (tx, ty). */
function LL4View() {
  const cfg = useNT<number[]>('/Sim/Vision/ll4/config', []);
  const frame = useNT<number[]>('/Sim/Vision/ll4/frame', []); // {count, tv, tx, ty, ta}
  const hfov = cfg[5] ?? 82, vfov = cfg[6] ?? 56;
  const W = 240, H = (W * vfov) / hfov;
  const tv = frame[1] > 0.5, tx = frame[2] ?? 0, ty = frame[3] ?? 0;
  const px = W / 2 + (tx / (hfov / 2)) * (W / 2);
  const py = H / 2 - (ty / (vfov / 2)) * (H / 2);
  return (
    <div className="ll4-view">
      <svg viewBox={`0 0 ${W} ${H}`}>
        <rect width={W} height={H} className="ll4-bg" />
        <line x1={W / 2} y1={0} x2={W / 2} y2={H} className="ll4-cross" />
        <line x1={0} y1={H / 2} x2={W} y2={H / 2} className="ll4-cross" />
        {tv && <rect x={px - 9} y={py - 9} width={18} height={18} className="ll4-target" />}
        {tv && <line x1={W / 2} y1={H / 2} x2={px} y2={py} className="ll4-offset" />}
        <text x={6} y={14} className="ll4-text">{tv ? `tx ${tx.toFixed(1)}°  ty ${ty.toFixed(1)}°` : 'no target'}</text>
      </svg>
      <p className="muted tiny">AutoAim turns the robot until tx ≈ 0 (within 1.5°).</p>
    </div>
  );
}

function Slider({ label, unit, min, max, step, value, onChange }: { label: string; unit: string; min: number; max: number; step: number; value: number; onChange: (v: number) => void }) {
  return (
    <label className="slider">
      <span className="slider-label">{label}</span>
      <input type="range" min={min} max={max} step={step} value={value} onChange={(e) => onChange(Number(e.target.value))} />
      <span className="slider-value">{Number.isInteger(step) ? value.toFixed(0) : value.toFixed(1)} {unit}</span>
    </label>
  );
}
