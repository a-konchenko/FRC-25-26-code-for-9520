// Field overlays for phases 3–4: camera view cones, sightlines to visible
// tags, vision measurements the robot code fused, and FUEL shots.
import { useEffect, useRef } from 'react';
import { getTags, type Variant } from '../../field/fieldData';
import { useNT, type Pose } from '../../nt/store';

interface Props {
  variant: Variant;
  truth: Pose | undefined;
  showCameras: boolean;
  showVision: boolean;
  showShots: boolean;
}

const FUSED_FADE_S = 2.5;

export function VisionLayer({ variant, truth, showCameras, showVision, showShots }: Props) {
  const ll3 = useNT<number[]>('/Sim/Vision/ll3/config', []);
  const ll4 = useNT<number[]>('/Sim/Vision/ll4/config', []);
  const ll3Vis = useNT<number[]>('/Sim/Vision/ll3/visible', []);
  const ll4Vis = useNT<number[]>('/Sim/Vision/ll4/visible', []);
  const ll3Frame = useNT<number[]>('/Sim/Vision/ll3/frame', []);
  const pvMounts = useNT<number[]>('/Sim/Vision/pv/mounts', []);
  const pvParams = useNT<number[]>('/Sim/Vision/pv/params', []);
  const pvVis = useNT<number[]>('/Sim/Vision/pv/visible', []);
  const fusedRaw = useNT<number[]>('/Sim/Vision/fused', []);
  const shots = useNT<number[]>('/Sim/Fuel/shots', []);
  const tags = getTags(variant);
  const tagById = new Map(tags.map((t) => [t.id, t]));

  // Fused measurements: remember when each (by sequence number) first arrived, so they fade
  const fused = useRef(new Map<number, { x: number; y: number; at: number }>());
  useEffect(() => {
    const now = performance.now() / 1000;
    for (let k = 0; k + 4 < fusedRaw.length; k += 5) {
      const seq = fusedRaw[k];
      if (!fused.current.has(seq)) fused.current.set(seq, { x: fusedRaw[k + 1], y: fusedRaw[k + 2], at: now });
    }
    for (const [seq, f] of fused.current) if (now - f.at > FUSED_FADE_S) fused.current.delete(seq);
  }, [fusedRaw]);
  const now = performance.now() / 1000;

  if (!truth) return null;

  // Camera on the field from the TRUE pose + its physical mount (configured + error)
  const camPose = (mx: number, my: number, yawDeg: number): [number, number, number] => {
    const c = Math.cos(truth[2]), s = Math.sin(truth[2]);
    return [truth[0] + mx * c - my * s, truth[1] + mx * s + my * c, truth[2] + (yawDeg * Math.PI) / 180];
  };
  const cones: { key: string; cam: [number, number, number]; hfov: number; range: number; vis: number[]; cls: string }[] = [];
  if (ll3.length >= 12 && ll3[8] > 0.5) cones.push({ key: 'll3', cam: camPose(ll3[0] + ll3[9], ll3[1] + ll3[10], ll3[4] + ll3[11]), hfov: ll3[5], range: ll3[7], vis: ll3Vis, cls: 'cam-ll3' });
  if (ll4.length >= 12 && ll4[8] > 0.5) cones.push({ key: 'll4', cam: camPose(ll4[0] + ll4[9], ll4[1] + ll4[10], ll4[4] + ll4[11]), hfov: ll4[5], range: ll4[7], vis: ll4Vis, cls: 'cam-ll4' });
  if (pvParams.length >= 7 && pvParams[3] > 0.5) {
    for (let i = 0; i * 6 + 5 < pvMounts.length; i++) {
      const m = pvMounts.slice(i * 6, i * 6 + 6);
      const vis: number[] = [];
      for (let k = 0; k + 1 < pvVis.length; k += 2) if (pvVis[k] === i) vis.push(pvVis[k + 1]);
      cones.push({ key: `pv${i}`, cam: camPose(m[0] + pvParams[4], m[1] + pvParams[5], m[5] + pvParams[6]), hfov: pvParams[0], range: pvParams[2], vis, cls: 'cam-pv' });
    }
  }

  return (
    <g className="vision-layer">
      {showCameras && cones.map((c) => {
        const r = Math.min(c.range, 3.5); // draw the near part of the cone; range is in the tooltip
        const a1 = c.cam[2] + (c.hfov * Math.PI) / 360, a2 = c.cam[2] - (c.hfov * Math.PI) / 360;
        const p = `${c.cam[0]},${c.cam[1]} ${c.cam[0] + Math.cos(a1) * r},${c.cam[1] + Math.sin(a1) * r} ${c.cam[0] + Math.cos(a2) * r},${c.cam[1] + Math.sin(a2) * r}`;
        return (
          <g key={c.key} className={`cam ${c.cls}`}>
            <title>{`${c.key.toUpperCase()}: ${c.hfov.toFixed(0)}° wide, sees tags up to ${c.range.toFixed(1)} m — now sees ${c.vis.length ? c.vis.join(', ') : 'no tags'}`}</title>
            <polygon points={p} className="cam-cone" />
            {c.vis.map((id) => {
              const t = tagById.get(id);
              return t ? <line key={id} x1={c.cam[0]} y1={c.cam[1]} x2={t.x} y2={t.y} className="cam-ray" /> : null;
            })}
            <circle cx={c.cam[0]} cy={c.cam[1]} r={0.035} className="cam-dot" />
          </g>
        );
      })}

      {showVision && ll3Frame.length >= 5 && (
        <g className="ll3-frame">
          <title>Latest Limelight 3 pose — what the camera says (before the robot code filters/blends it)</title>
          <circle cx={ll3Frame[1]} cy={ll3Frame[2]} r={0.09} />
          <line x1={ll3Frame[1]} y1={ll3Frame[2]} x2={ll3Frame[1] + Math.cos(ll3Frame[3]) * 0.18} y2={ll3Frame[2] + Math.sin(ll3Frame[3]) * 0.18} />
        </g>
      )}
      {showVision && [...fused.current.entries()].map(([seq, f]) => (
        <circle key={seq} cx={f.x} cy={f.y} r={0.045} className="fused-dot" style={{ opacity: Math.max(0, 1 - (now - f.at) / FUSED_FADE_S) }}>
          <title>Vision measurement the robot code fused into its pose estimate</title>
        </circle>
      ))}

      {showShots && Array.from({ length: Math.floor(shots.length / 6) }, (_, i) => {
        const s = shots.slice(i * 6, i * 6 + 6);
        const hit = s[5] === 0;
        return (
          <g key={s[0]} className={`shot ${hit ? 'shot-hit' : 'shot-miss'}`}>
            <line x1={s[1]} y1={s[2]} x2={s[3]} y2={s[4]} markerEnd="url(#arrow)" />
            {!hit && <circle cx={s[3]} cy={s[4]} r={0.06} />}
          </g>
        );
      })}
    </g>
  );
}
