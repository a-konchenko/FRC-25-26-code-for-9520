// Localization error over time (Truth vs Estimate). Plain SVG, no chart lib.
export interface ErrorSample { t: number; loc: number; head: number; }

const W = 1000, H = 120, PAD_L = 44, PAD_R = 44, PAD_T = 10, PAD_B = 22;

export function ErrorChart({ samples }: { samples: ErrorSample[] }) {
  const n = samples.length;
  const t1 = n ? samples[n - 1].t : 0;
  const t0 = Math.max(n ? samples[0].t : 0, t1 - 30);
  const span = Math.max(t1 - t0, 1);
  const vis = samples.filter((s) => s.t >= t0);
  const maxLoc = Math.max(5, ...vis.map((s) => s.loc * 100)) * 1.15;
  const maxHead = Math.max(1, ...vis.map((s) => Math.abs(s.head))) * 1.15;

  const x = (t: number) => PAD_L + ((t - t0) / span) * (W - PAD_L - PAD_R);
  const yLoc = (cm: number) => H - PAD_B - (cm / maxLoc) * (H - PAD_T - PAD_B);
  const yHead = (deg: number) => H - PAD_B - (Math.abs(deg) / maxHead) * (H - PAD_T - PAD_B);
  const loc = vis.map((s) => `${x(s.t).toFixed(1)},${yLoc(s.loc * 100).toFixed(1)}`).join(' ');
  const head = vis.map((s) => `${x(s.t).toFixed(1)},${yHead(s.head).toFixed(1)}`).join(' ');

  return (
    <div className="chart">
      <div className="chart-title">
        <span><i className="swatch swatch-loc" />Position error (cm)</span>
        <span><i className="swatch swatch-head" />Heading error (°)</span>
        <span className="muted">last 30 s · localization = |estimate − truth| → a sensing problem</span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`}>
        {[0, 0.5, 1].map((f) => (
          <g key={f}>
            <line x1={PAD_L} x2={W - PAD_R} y1={yLoc(maxLoc * f)} y2={yLoc(maxLoc * f)} className="chart-grid" />
            <text x={PAD_L - 6} y={yLoc(maxLoc * f)} className="chart-axis" textAnchor="end" dominantBaseline="central">{(maxLoc * f).toFixed(0)}</text>
            <text x={W - PAD_R + 6} y={yHead(maxHead * f)} className="chart-axis" dominantBaseline="central">{(maxHead * f).toFixed(1)}</text>
          </g>
        ))}
        {vis.length > 1 && <polyline points={head} className="chart-line chart-head" />}
        {vis.length > 1 && <polyline points={loc} className="chart-line chart-loc" />}
        {!vis.length && <text x={W / 2} y={H / 2} textAnchor="middle" className="chart-axis">waiting for data…</text>}
      </svg>
    </div>
  );
}
