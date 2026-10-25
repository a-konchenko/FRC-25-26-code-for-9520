// Velocity vs time for a previewed path, with a cursor at the animation time.
export interface PreviewState {
  exact: boolean;      // from PathPlannerLib (robot sim) vs approximate
  pending: boolean;
  total: number;       // s
  states: number[][];  // [t, x, y, θ, v]
  start: number;       // performance.now() when the animation started
}

const W = 1000, H = 110, PL = 44, PR = 16, PT = 10, PB = 22;

export function PreviewChart({ preview, t }: { preview: PreviewState; t: number }) {
  const total = Math.max(preview.total, 0.1);
  const vmax = Math.max(0.5, ...preview.states.map((s) => s[4])) * 1.1;
  const x = (tt: number) => PL + (tt / total) * (W - PL - PR);
  const y = (v: number) => H - PB - (v / vmax) * (H - PT - PB);
  const pts = preview.states.map((s) => `${x(s[0]).toFixed(1)},${y(s[4]).toFixed(1)}`).join(' ');
  return (
    <div className="chart">
      <div className="chart-title">
        <span><i className="swatch swatch-plan" />Velocity (m/s)</span>
        <span><b>{preview.total.toFixed(2)} s</b> total</span>
        <span className="muted">{preview.exact ? 'exact — PathPlannerLib with settings.json, same as the robot' : 'approximate — start the robot sim for exact timing'}</span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`}>
        {[0, 0.5, 1].map((f) => (
          <g key={f}>
            <line x1={PL} x2={W - PR} y1={y(vmax * f)} y2={y(vmax * f)} className="chart-grid" />
            <text x={PL - 6} y={y(vmax * f)} className="chart-axis" textAnchor="end" dominantBaseline="central">{(vmax * f).toFixed(1)}</text>
          </g>
        ))}
        {[0, 0.25, 0.5, 0.75, 1].map((f) => (
          <text key={f} x={x(total * f)} y={H - 6} className="chart-axis" textAnchor="middle">{(total * f).toFixed(1)}s</text>
        ))}
        <polyline points={pts} className="chart-line chart-plan" />
        <line x1={x(Math.min(t, total))} x2={x(Math.min(t, total))} y1={PT} y2={H - PB} className="chart-cursor" />
      </svg>
    </div>
  );
}
