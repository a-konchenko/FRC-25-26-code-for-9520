// Robot drawn in field coordinates (inside FieldView).
import type { Pose } from '../nt/store';

export type RobotStyle = 'truth' | 'estimate' | 'setpoint' | 'ghost' | 'placing' | 'plan-start' | 'plan-end' | 'plan-rot' | 'plan-rot-active' | 'preview' | 'replay-truth' | 'replay-estimate';

interface Props {
  pose: Pose;
  /** [length with bumpers, width with bumpers, frame length, frame width] (m) */
  size: number[];
  style: RobotStyle;
  alliance?: 'blue' | 'red';
  /** Module positions [x0,y0,x1,y1,...] (m, robot frame) */
  moduleLocations?: number[];
  /** Module states [angle0, speed0, ...] (rad, m/s) */
  modules?: number[];
  maxSpeed?: number;
  title?: string;
}

export function RobotShape({ pose, size, style, alliance = 'blue', moduleLocations, modules, maxSpeed = 4.4, title }: Props) {
  const [x, y, th] = pose;
  const [L, W, fL, fW] = size.length >= 4 ? size : [0.85, 0.78, 0.69, 0.61];
  const deg = (th * 180) / Math.PI;
  const bumper = (L - fL) / 2;

  return (
    <g transform={`translate(${x} ${y}) rotate(${deg})`} className={`robot robot-${style} robot-${alliance}`}>
      {title && <title>{title}</title>}
      {/* bumpers */}
      <rect x={-L / 2} y={-W / 2} width={L} height={W} rx={0.05} className="robot-bumper" />
      {/* frame */}
      {style === 'truth' && (
        <rect x={-fL / 2} y={-fW / 2} width={fL} height={fW} className="robot-frame" />
      )}
      {/* front marker: the bumper face that is "forward" */}
      <line x1={L / 2 - bumper / 2} y1={-W / 2 + 0.06} x2={L / 2 - bumper / 2} y2={W / 2 - 0.06} className="robot-front" />
      {/* heading arrow */}
      <line x1={0} y1={0} x2={L / 2 - 0.08} y2={0} className="robot-heading" markerEnd="url(#arrow)" />

      {/* swerve modules: wheel at the real steer angle, length ∝ speed */}
      {style === 'truth' && moduleLocations && modules &&
        Array.from({ length: Math.min(moduleLocations.length / 2, modules.length / 2) }, (_, i) => {
          const mx = moduleLocations[2 * i], my = moduleLocations[2 * i + 1];
          const a = (modules[2 * i] * 180) / Math.PI;
          const sp = modules[2 * i + 1];
          const len = 0.05 + Math.min(Math.abs(sp) / maxSpeed, 1) * 0.12;
          return (
            <g key={i} transform={`translate(${mx} ${my}) rotate(${a})`}>
              <rect x={-0.05} y={-0.022} width={0.1} height={0.044} className="robot-wheel" />
              {Math.abs(sp) > 0.05 && (
                <line x1={0} y1={0} x2={Math.sign(sp) * len} y2={0} className="robot-wheel-vec" markerEnd="url(#arrow)" />
              )}
            </g>
          );
        })}
    </g>
  );
}

/** Path trace (polyline) in field coordinates. */
export function Trace({ points, className }: { points: [number, number][]; className: string }) {
  if (points.length < 2) return null;
  return <polyline points={points.map((p) => `${p[0].toFixed(3)},${p[1].toFixed(3)}`).join(' ')} className={className} />;
}
