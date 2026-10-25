// SVG layer (inside FieldView) that draws an editable PathPlanner path.
import { FieldText } from '../../field/FieldView';
import { RobotShape } from '../../field/RobotShape';
import { pointAt, type PathSample } from '../../pathplanner/geometry';
import type { Collision } from '../../pathplanner/collision';
import type { PPPath, XY } from '../../pathplanner/types';
import type { Selection } from './selection';

export const HANDLE_R = 0.09;

/** Where the rotation handle of a robot outline sits (in front of the bumper). */
export function rotationHandle(at: XY, rad: number, robotLength: number): XY {
  const r = robotLength / 2 + 0.22;
  return { x: at.x + Math.cos(rad) * r, y: at.y + Math.sin(rad) * r };
}

interface Props {
  path: PPPath;
  samples: PathSample[];
  sel: Selection;
  size: number[];          // [L, W, frameL, frameW]
  collisions: Collision[];
  showFootprint: boolean;
  dim?: boolean;
}

export function PathLayer({ path, samples, sel, size, collisions, showFootprint, dim }: Props) {
  const L = size[0] ?? 0.85;
  const wps = path.waypoints;
  const n = wps.length - 1;
  const deg = (d: number) => (d * Math.PI) / 180;
  const startRot = deg(path.idealStartingState?.rotation ?? 0);
  const endRot = deg(path.goalEndState?.rotation ?? 0);
  const pts = samples.map((s) => `${s.x.toFixed(3)},${s.y.toFixed(3)}`).join(' ');
  const selectedWp = sel && (sel.type === 'anchor' || sel.type === 'prev' || sel.type === 'next') ? sel.i : -1;

  // footprint ghosts every ~0.6 m
  const ghosts: PathSample[] = [];
  if (showFootprint) {
    let acc = Infinity, last: PathSample | null = null;
    for (const s of samples) {
      if (last) acc += Math.hypot(s.x - last.x, s.y - last.y);
      if (acc >= 0.6) { ghosts.push(s); acc = 0; }
      last = s;
    }
  }

  return (
    <g className={`plan-layer ${dim ? 'plan-dim' : ''}`}>
      {ghosts.map((g, k) => <RobotShape key={k} pose={[g.x, g.y, g.rot]} size={size} style="ghost" />)}
      <polyline points={pts} className="plan-curve" />
      {collisions.length > 0 && collisions.map((c, k) => <circle key={k} cx={c.x} cy={c.y} r={0.07} className="plan-collision"><title>{`Bumpers hit ${c.what}`}</title></circle>)}

      {/* start / end robots + rotation handles */}
      <RobotShape pose={[wps[0].anchor.x, wps[0].anchor.y, startRot]} size={size} style="plan-start" />
      <RobotShape pose={[wps[n].anchor.x, wps[n].anchor.y, endRot]} size={size} style="plan-end" />
      {!dim && <RotHandle at={rotationHandle(wps[0].anchor, startRot, L)} from={wps[0].anchor} active={sel?.type === 'start'} cls="start" />}
      {!dim && <RotHandle at={rotationHandle(wps[n].anchor, endRot, L)} from={wps[n].anchor} active={sel?.type === 'end'} cls="end" />}

      {/* rotation targets */}
      {path.rotationTargets.map((r, k) => {
        const at = pointAt(path, r.waypointRelativePos);
        const rad = deg(r.rotationDegrees);
        const active = sel?.type === 'rot' && sel.i === k;
        return (
          <g key={`rt${k}`}>
            <RobotShape pose={[at.x, at.y, rad]} size={size} style={active ? 'plan-rot-active' : 'plan-rot'} />
            {!dim && <circle cx={at.x} cy={at.y} r={HANDLE_R * 0.8} className={`plan-rot-dot ${active ? 'active' : ''}`} />}
            {!dim && <RotHandle at={rotationHandle(at, rad, L)} from={at} active={active} cls="rot" />}
          </g>
        );
      })}

      {/* event markers */}
      {path.eventMarkers.map((m, k) => {
        const at = pointAt(path, m.waypointRelativePos);
        const active = sel?.type === 'marker' && sel.i === k;
        const s = 0.11;
        return (
          <g key={`em${k}`} className={`plan-marker ${active ? 'active' : ''}`}>
            <title>{`${m.name}${m.command?.data?.name ? ` → ${m.command.data.name}` : ''}`}</title>
            <polygon points={`${at.x},${at.y + s} ${at.x + s},${at.y} ${at.x},${at.y - s} ${at.x - s},${at.y}`} />
            <FieldText x={at.x} y={at.y + 0.24} className="plan-marker-label">{m.command?.data?.name ?? m.name}</FieldText>
          </g>
        );
      })}

      {/* waypoints + control handles */}
      {!dim && wps.map((w, i) => (
        <g key={`wp${i}`}>
          {i === selectedWp && w.prevControl && <Handle at={w.prevControl} from={w.anchor} active={sel?.type === 'prev'} />}
          {i === selectedWp && w.nextControl && <Handle at={w.nextControl} from={w.anchor} active={sel?.type === 'next'} />}
          <circle cx={w.anchor.x} cy={w.anchor.y} r={HANDLE_R} className={`plan-anchor ${i === selectedWp ? 'active' : ''} ${i === 0 ? 'first' : i === n ? 'last' : ''}`} />
          <FieldText x={w.anchor.x} y={w.anchor.y} className="plan-anchor-label">{i}</FieldText>
        </g>
      ))}
    </g>
  );
}

function Handle({ at, from, active }: { at: XY; from: XY; active: boolean }) {
  return (
    <g className={`plan-handle ${active ? 'active' : ''}`}>
      <line x1={from.x} y1={from.y} x2={at.x} y2={at.y} />
      <circle cx={at.x} cy={at.y} r={HANDLE_R * 0.75} />
    </g>
  );
}

function RotHandle({ at, from, active, cls }: { at: XY; from: XY; active: boolean; cls: string }) {
  return (
    <g className={`plan-rothandle plan-rothandle-${cls} ${active ? 'active' : ''}`}>
      <line x1={from.x} y1={from.y} x2={at.x} y2={at.y} />
      <circle cx={at.x} cy={at.y} r={HANDLE_R * 0.7} />
    </g>
  );
}
