// Field overlay for phase 5a: the FUEL lying on the field (physics runs in
// the robot sim, FieldFuel.java) and the robot's intake zone.
import { useNT, type Pose } from '../../nt/store';

const R = (5.91 * 0.0254) / 2;

interface Props {
  truth: Pose | undefined;
  /** [length with bumpers, width with bumpers, ...] */
  size: number[];
  showFuel: boolean;
}

export function FuelLayer({ truth, size, showFuel }: Props) {
  const balls = useNT<number[]>('/Sim/Fuel/field', []);
  const intake = useNT<number[]>('/Sim/Robot/intake', []); // {depth, width}
  const mech = useNT<number[]>('/Sim/Mech/state', []);    // [3] = intake roller rps
  const intakeOn = Math.abs(mech[3] ?? 0) > 5;

  // All balls in ONE path element: 500 circles redrawn every frame stays cheap
  let d = '';
  if (showFuel) {
    for (let i = 0; i + 1 < balls.length; i += 2) {
      const x = balls[i], y = balls[i + 1];
      d += `M${(x + R).toFixed(3)},${y.toFixed(3)}a${R},${R} 0 1,0 ${-2 * R},0a${R},${R} 0 1,0 ${2 * R},0`;
    }
  }

  const L = size[0] ?? 0.85;
  return (
    <g className="fuel-layer">
      {d && <path d={d} className="field-fuel"><title>FUEL on the field (5.91 in foam balls)</title></path>}
      {truth && intake.length >= 2 && (
        <g transform={`translate(${truth[0]} ${truth[1]}) rotate(${(truth[2] * 180) / Math.PI})`}>
          <rect x={L / 2} y={-intake[1] / 2} width={intake[0]} height={intake[1]} className={`intake-zone ${intakeOn ? 'on' : ''}`}>
            <title>{`Intake ${intakeOn ? 'running' : 'off'}: FUEL touching this zone is picked up while the roller spins`}</title>
          </rect>
        </g>
      )}
    </g>
  );
}
