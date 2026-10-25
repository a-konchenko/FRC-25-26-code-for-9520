// Plan-vs-fact errors over a run: tracking (setpoint vs estimate) and
// localization (estimate vs truth), with a cursor at the scrubbed time.
export interface RunFrame {
  t: number;
  truth: [number, number, number];
  est: [number, number, number];
  sp: [number, number, number] | null;
}

const W = 1000, H = 130, PL = 44, PR = 16, PT = 10, PB = 22;

export function trackingErr(f: RunFrame): number | null {
  return f.sp ? Math.hypot(f.sp[0] - f.est[0], f.sp[1] - f.est[1]) : null;
}
export function locErr(f: RunFrame): number {
  return Math.hypot(f.est[0] - f.truth[0], f.est[1] - f.truth[1]);
}

export function RunChart({ frames, duration, cursor, onScrub }: {
  frames: RunFrame[];
  duration: number;
  cursor: number | null;
  onScrub: (t: number) => void;
}) {
  const track = frames.map((f) => ({ t: f.t, v: trackingErr(f) }));
  const loc = frames.map((f) => ({ t: f.t, v: locErr(f) }));
  const vmax = Math.max(5, ...track.map((p) => (p.v ?? 0) * 100), ...loc.map((p) => p.v * 100)) * 1.15;
  const x = (t: number) => PL + (t / duration) * (W - PL - PR);
  const y = (cm: number) => H - PB - (cm / vmax) * (H - PT - PB);

  // tracking error only exists while a path is being followed → break the line in gaps
  const trackSegs: string[] = [];
  let cur: string[] = [];
  for (const p of track) {
    if (p.v == null) { if (cur.length > 1) trackSegs.push(cur.join(' ')); cur = []; continue; }
    cur.push(`${x(p.t).toFixed(1)},${y(p.v * 100).toFixed(1)}`);
  }
  if (cur.length > 1) trackSegs.push(cur.join(' '));
  const locPts = loc.map((p) => `${x(p.t).toFixed(1)},${y(p.v * 100).toFixed(1)}`).join(' ');

  const scrub = (e: React.PointerEvent<SVGSVGElement>) => {
    if (e.type === 'pointermove' && !e.buttons) return;
    const r = e.currentTarget.getBoundingClientRect();
    const px = ((e.clientX - r.left) / r.width) * W;
    onScrub(Math.min(Math.max(((px - PL) / (W - PL - PR)) * duration, 0), duration));
  };

  return (
    <div className="chart">
      <div className="chart-title">
        <span title="Setpoint vs estimate: is the path follower keeping up? A controller / tuning problem."><i className="swatch swatch-track" />Tracking error (cm)</span>
        <span title="Estimate vs truth: does the robot know where it is? A sensing problem (drift, cameras)."><i className="swatch swatch-loc" />Localization error (cm)</span>
        <span className="muted">click or drag to scrub the run</span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} onPointerDown={scrub} onPointerMove={scrub} className="scrubbable">
        {[0, 0.5, 1].map((f) => (
          <g key={f}>
            <line x1={PL} x2={W - PR} y1={y(vmax * f)} y2={y(vmax * f)} className="chart-grid" />
            <text x={PL - 6} y={y(vmax * f)} className="chart-axis" textAnchor="end" dominantBaseline="central">{(vmax * f).toFixed(0)}</text>
          </g>
        ))}
        {Array.from({ length: Math.floor(duration / 5) + 1 }, (_, k) => k * 5).map((t) => (
          <text key={t} x={x(t)} y={H - 6} className="chart-axis" textAnchor="middle">{t}s</text>
        ))}
        {locPts && <polyline points={locPts} className="chart-line chart-loc" />}
        {trackSegs.map((s, k) => <polyline key={k} points={s} className="chart-line chart-track" />)}
        {cursor != null && <line x1={x(cursor)} x2={x(cursor)} y1={PT} y2={H - PB} className="chart-cursor" />}
        {!frames.length && <text x={W / 2} y={H / 2} textAnchor="middle" className="chart-axis">press ▶ Play to record a run</text>}
      </svg>
    </div>
  );
}
